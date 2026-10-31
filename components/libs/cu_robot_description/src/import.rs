use cu_collision::CuCapsule;
use cu_collision::CuCollisionModelParams;
use cu_collision::CuCollisionShape;
use cu_collision::CuLinkShape;
use cu_collision::CuRobotCollisionModel;
use cu_collision::CuSphere;
use cu_kinematics::CuJointAxis;
use cu_kinematics::CuJointLimits;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointSpec;
use cu_kinematics::CuKinematicChain;
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::AngularVelocity;
use cu29::units::si::f64::Length;
use cu29::units::si::f64::Velocity;
use cu29::units::si::length::meter;
use cu29::units::si::velocity::meter_per_second;
use roxmltree::Document;
use roxmltree::Node;
use std::collections::BTreeMap;
use std::collections::BTreeSet;

/// Collision parameters used when an import does not override them: zero safety margin and
/// **no** adjacent-link skipping. URDF import relies on the SRDF `disable_collisions` list to
/// disable pairs, so every pair of shapes on different frames starts enabled.
fn default_import_collision() -> CuCollisionModelParams {
    CuCollisionModelParams {
        safety_margin: Length::new::<meter>(0.0),
        skip_adjacent_links: false,
    }
}

/// Named inputs of [`CuRobotDescription::from_urdf`].
///
/// ```ignore
/// CuUrdfImport::new("base", "tool").with_collision(CuCollisionModelParams {
///     safety_margin: Length::new::<meter>(0.005),
///     skip_adjacent_links: false,
/// })
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CuUrdfImport<'a> {
    /// URDF link where the serial chain starts.
    pub base_link: &'a str,
    /// URDF link where the serial chain ends (the tool frame).
    pub tip_link: &'a str,
    /// Collision model parameters. Default: zero margin, `skip_adjacent_links: false`.
    pub collision: CuCollisionModelParams,
}

impl<'a> CuUrdfImport<'a> {
    /// Selects the `base_link` to `tip_link` path with default collision parameters.
    pub fn new(base_link: &'a str, tip_link: &'a str) -> Self {
        Self {
            base_link,
            tip_link,
            collision: default_import_collision(),
        }
    }

    /// Replaces the collision model parameters.
    pub const fn with_collision(mut self, collision: CuCollisionModelParams) -> Self {
        self.collision = collision;
        self
    }
}

/// Named inputs of [`CuRobotDescription::from_urdf_srdf`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CuSrdfImport<'a> {
    /// SRDF planning group containing exactly one serial chain.
    pub group: &'a str,
    /// Collision model parameters. Default: zero margin, `skip_adjacent_links: false`.
    pub collision: CuCollisionModelParams,
}

impl<'a> CuSrdfImport<'a> {
    /// Selects SRDF `group` with default collision parameters.
    pub fn new(group: &'a str) -> Self {
        Self {
            group,
            collision: default_import_collision(),
        }
    }

    /// Replaces the collision model parameters.
    pub const fn with_collision(mut self, collision: CuCollisionModelParams) -> Self {
        self.collision = collision;
        self
    }
}

/// A startup robot description for a single base-to-tip serial chain.
/// `N` counts movable joints; `M` bounds collision shapes, including fixed and base links.
/// Names and semantic state storage allocate only during import and lookup.
///
/// The runtime models are read through [`chain`](Self::chain) and
/// [`collision_model`](Self::collision_model) (both `Copy`), or taken with
/// [`into_parts`](Self::into_parts).
#[derive(Debug)]
pub struct CuRobotDescription<const N: usize, const M: usize> {
    chain: CuKinematicChain<N>,
    collision_model: CuRobotCollisionModel<N, M>,
    joint_names: [String; N],
    shape_links: [String; M],
    links: BTreeSet<String>,
    all_links: BTreeSet<String>,
    robot_name: String,
    base: String,
    tip: String,
    named_states: BTreeMap<String, [CuJointPosition; N]>,
}

impl<const N: usize, const M: usize> CuRobotDescription<N, M> {
    /// Imports an explicit URDF base-to-tip path. Other branches are not part of this model.
    /// Rejects unsupported selected joints and geometry rather than silently omitting them.
    pub fn from_urdf(xml: &str, import: CuUrdfImport<'_>) -> CuResult<Self> {
        let CuUrdfImport {
            base_link: base,
            tip_link: tip,
            collision,
        } = import;
        collision.validate()?;
        let doc =
            Document::parse(xml).map_err(|e| CuError::from(format!("Invalid URDF XML: {e}")))?;
        let robot = robot_root(&doc)?;
        let mut links = BTreeMap::new();
        let mut joints = BTreeMap::new();
        let mut parents = BTreeMap::new();
        for node in robot.children().filter(Node::is_element) {
            if node.has_tag_name("link") {
                let name = attr(node, "name")?;
                if links.insert(name, node).is_some() {
                    return Err(CuError::from("Duplicate URDF link name"));
                }
            } else if node.has_tag_name("joint") {
                let name = attr(node, "name")?;
                if joints.insert(name, node).is_some() {
                    return Err(CuError::from("Duplicate URDF joint name"));
                }
                let child = attr(child(node, "child")?, "link")?;
                if parents.insert(child, node).is_some() {
                    return Err(CuError::from("URDF link has multiple parent joints"));
                }
            }
        }
        if !links.contains_key(base) || !links.contains_key(tip) || base == tip {
            return Err(CuError::from(
                "Expected distinct existing base and tip links",
            ));
        }
        for node in joints.values() {
            let p = attr(child(*node, "parent")?, "link")?;
            let c = attr(child(*node, "child")?, "link")?;
            if !links.contains_key(p) || !links.contains_key(c) {
                return Err(CuError::from("URDF joint references unknown link"));
            }
        }
        if links
            .keys()
            .filter(|name| !parents.contains_key(**name))
            .count()
            != 1
        {
            return Err(CuError::from("URDF must have exactly one root link"));
        }
        // Validate tree ancestry, including the ancestors of the requested base.
        for name in links.keys() {
            let mut seen = BTreeSet::new();
            let mut current = *name;
            while let Some(joint) = parents.get(current) {
                if !seen.insert(current) {
                    return Err(CuError::from("Cycle in URDF joint topology"));
                }
                current = attr(child(*joint, "parent")?, "link")?;
            }
        }
        let mut path = Vec::new();
        let mut current = tip;
        while current != base {
            let joint = *parents
                .get(current)
                .ok_or_else(|| CuError::from("Tip is not a descendant of base"))?;
            path.push(joint);
            current = attr(child(joint, "parent")?, "link")?;
        }
        path.reverse();
        let mut specs = [CuJointSpec::default(); N];
        let mut joint_names = core::array::from_fn(|_| String::new());
        let mut pending = Transform3D::<f64>::identity();
        let mut count = 0;
        let mut owner = None;
        let mut link_frames = vec![(base, owner, pending)];
        for node in path {
            if node.children().any(|n| n.has_tag_name("mimic")) {
                return Err(CuError::from(
                    "Mimic joints are unsupported in a serial chain",
                ));
            }
            pending = pending.compose(origin(node)?);
            let kind = attr(node, "type")?;
            if !matches!(kind, "fixed" | "revolute" | "continuous" | "prismatic") {
                return Err(CuError::from(
                    "Unsupported URDF joint type in selected serial chain",
                ));
            }
            if kind != "fixed" {
                if count >= N {
                    return Err(CuError::from("URDF movable joint count exceeds N"));
                }
                let axis = if let Some(axis) = optional_child(node, "axis")? {
                    vector(attr(axis, "xyz")?)?
                } else {
                    [1.0, 0.0, 0.0]
                };
                let norm = axis.iter().map(|v| v * v).sum::<f64>().sqrt();
                if norm <= 1e-12 || !norm.is_finite() {
                    return Err(CuError::from("URDF joint axis must be nonzero and finite"));
                }
                let limits_node = child(node, "limit")?;
                let velocity = number(attr(limits_node, "velocity")?)?;
                if velocity <= 0.0 {
                    return Err(CuError::from("URDF joint velocity limit must be positive"));
                }
                let (lower, upper) = if kind == "continuous" {
                    // A finite canonical sampling interval; FK accepts angles outside it.
                    (-core::f64::consts::PI, core::f64::consts::PI)
                } else {
                    (
                        number(attr(limits_node, "lower")?)?,
                        number(attr(limits_node, "upper")?)?,
                    )
                };
                if lower > upper {
                    return Err(CuError::from("URDF lower limit exceeds upper limit"));
                }
                let (xyz, rpy) = decompose(pending);
                let xyz = xyz.map(Length::new::<meter>);
                let rpy = rpy.map(Angle::new::<radian>);
                let axis = CuJointAxis::Arbitrary(axis.map(|v| v / norm));
                specs[count] = match kind {
                    "revolute" => CuJointSpec::revolute(
                        xyz,
                        rpy,
                        axis,
                        CuJointLimits::revolute(
                            Angle::new::<radian>(lower),
                            Angle::new::<radian>(upper),
                            AngularVelocity::new::<radian_per_second>(velocity),
                        ),
                    ),
                    "continuous" => CuJointSpec::revolute(
                        xyz,
                        rpy,
                        axis,
                        CuJointLimits::Continuous {
                            max_velocity: AngularVelocity::new::<radian_per_second>(velocity),
                        },
                    ),
                    "prismatic" => CuJointSpec::prismatic(
                        xyz,
                        rpy,
                        axis,
                        CuJointLimits::prismatic(
                            Length::new::<meter>(lower),
                            Length::new::<meter>(upper),
                            Velocity::new::<meter_per_second>(velocity),
                        ),
                    ),
                    _ => {
                        return Err(CuError::from(
                            "Only fixed, revolute, continuous, and prismatic URDF joints are supported",
                        ));
                    }
                };
                joint_names[count] = attr(node, "name")?.to_owned();
                owner = Some(count);
                count += 1;
                pending = Transform3D::<f64>::identity();
            }
            link_frames.push((attr(child(node, "child")?, "link")?, owner, pending));
        }
        if count != N {
            return Err(CuError::from("URDF movable joint count does not match N"));
        }
        let chain = CuKinematicChain::new(specs).with_tool_offset(pending);
        let mut shapes = [CuLinkShape::default(); M];
        let mut shape_links = core::array::from_fn(|_| String::new());
        let mut len = 0;
        for (link, frame, offset) in &link_frames {
            for collision in links[link]
                .children()
                .filter(|n| n.has_tag_name("collision"))
            {
                if len >= M {
                    return Err(CuError::from("URDF collision shape count exceeds M"));
                }
                let geometry = child(collision, "geometry")?;
                let mut elements = geometry.children().filter(Node::is_element);
                let shape = elements
                    .next()
                    .ok_or_else(|| CuError::from("Empty URDF collision geometry"))?;
                if elements.next().is_some() {
                    return Err(CuError::from(
                        "Expected exactly one URDF geometry primitive",
                    ));
                }
                let m = offset.compose(origin(collision)?).to_matrix();
                let shape = if shape.has_tag_name("sphere") {
                    let radius = positive(attr(shape, "radius")?)?;
                    CuCollisionShape::Sphere(CuSphere::new(
                        point(&m, [0.0; 3]),
                        Length::new::<meter>(radius),
                    )?)
                } else if shape.has_tag_name("cylinder") {
                    let radius = positive(attr(shape, "radius")?)?;
                    let half = positive(attr(shape, "length")?)? * 0.5;
                    // Conservative capsule encloses the full cylinder, including its end faces.
                    CuCollisionShape::Capsule(CuCapsule::new(
                        point(&m, [0.0, 0.0, -half]),
                        point(&m, [0.0, 0.0, half]),
                        Length::new::<meter>(radius),
                    )?)
                } else {
                    return Err(CuError::from(
                        "Unsupported collision geometry: provide spheres/cylinders; preprocess boxes and meshes explicitly",
                    ));
                };
                shapes[len] = CuLinkShape::new(shape, *frame);
                shape_links[len] = (*link).to_owned();
                len += 1;
            }
        }
        let collision_model = CuRobotCollisionModel::new(&shapes[..len], collision)?;
        Ok(Self {
            chain,
            collision_model,
            joint_names,
            shape_links,
            links: link_frames
                .iter()
                .map(|(name, _, _)| (*name).to_owned())
                .collect(),
            all_links: links.keys().map(|name| (*name).to_owned()).collect(),
            robot_name: attr(robot, "name")?.to_owned(),
            base: base.to_owned(),
            tip: tip.to_owned(),
            named_states: BTreeMap::new(),
        })
    }

    /// Serial kinematic chain from the base link to the tip link.
    pub const fn chain(&self) -> &CuKinematicChain<N> {
        &self.chain
    }
    /// Collision model (including SRDF-disabled pairs once applied).
    pub const fn collision_model(&self) -> &CuRobotCollisionModel<N, M> {
        &self.collision_model
    }
    /// Consumes the description and returns the runtime chain and collision model.
    pub fn into_parts(self) -> (CuKinematicChain<N>, CuRobotCollisionModel<N, M>) {
        (self.chain, self.collision_model)
    }
    /// Joint names in the exact order used by runtime payloads.
    pub fn joint_names(&self) -> &[String; N] {
        &self.joint_names
    }
    /// URDF link name for each active collision shape, in contact-index order.
    pub fn collision_links(&self) -> &[String] {
        &self.shape_links[..self.collision_model.shape_count()]
    }

    /// Selects an SRDF group containing exactly one serial base-to-tip chain.
    pub fn from_urdf_srdf(urdf: &str, srdf: &str, import: CuSrdfImport<'_>) -> CuResult<Self> {
        let CuSrdfImport { group, collision } = import;
        let doc =
            Document::parse(srdf).map_err(|e| CuError::from(format!("Invalid SRDF XML: {e}")))?;
        let robot = robot_root(&doc)?;
        let selected = named_child(robot, "group", group)?;
        let chain = serial_group(selected)?;
        let mut out = Self::from_urdf(
            urdf,
            CuUrdfImport::new(attr(chain, "base_link")?, attr(chain, "tip_link")?)
                .with_collision(collision),
        )?;
        out.apply_srdf(srdf, group)?;
        Ok(out)
    }

    /// Applies disabled collision pairs and complete named joint states for the selected serial group.
    /// Unsupported selected-group semantics fail explicitly; other groups are not imported.
    pub fn apply_srdf(&mut self, xml: &str, group: &str) -> CuResult<()> {
        let doc =
            Document::parse(xml).map_err(|e| CuError::from(format!("Invalid SRDF XML: {e}")))?;
        let robot = robot_root(&doc)?;
        if attr(robot, "name")? != self.robot_name {
            return Err(CuError::from("URDF and SRDF robot names differ"));
        }
        let chain = serial_group(named_child(robot, "group", group)?)?;
        if attr(chain, "base_link")? != self.base || attr(chain, "tip_link")? != self.tip {
            return Err(CuError::from(
                "SRDF group does not match imported serial chain",
            ));
        }
        // Stage changes so invalid semantics do not partially modify the model.
        let mut collision_model = self.collision_model;
        let mut states = BTreeMap::new();
        for node in robot.children().filter(Node::is_element) {
            if node.has_tag_name("disable_collisions") {
                let a = attr(node, "link1")?;
                let b = attr(node, "link2")?;
                if !self.all_links.contains(a) || !self.all_links.contains(b) || a == b {
                    return Err(CuError::from("Invalid SRDF disabled collision pair"));
                }
                // Pairs involving other URDF branches have no effect on this selected model.
                if self.links.contains(a) && self.links.contains(b) {
                    for i in 0..self.collision_model.shape_count() {
                        for j in i + 1..self.collision_model.shape_count() {
                            if (self.shape_links[i] == a && self.shape_links[j] == b)
                                || (self.shape_links[i] == b && self.shape_links[j] == a)
                            {
                                collision_model.set_pair_enabled(i, j, false)?;
                            }
                        }
                    }
                }
            } else if node.has_tag_name("group_state") && attr(node, "group")? == group {
                let name = attr(node, "name")?;
                let mut positions = core::array::from_fn(|i| self.chain.joints()[i].limits.lower());
                let mut seen = [false; N];
                for joint in node.children().filter(Node::is_element) {
                    if !joint.has_tag_name("joint") {
                        return Err(CuError::from("Unsupported SRDF group state element"));
                    }
                    let joint_name = attr(joint, "name")?;
                    let index = self
                        .joint_names
                        .iter()
                        .position(|n| n == joint_name)
                        .ok_or_else(|| {
                            CuError::from("SRDF state references joint outside selected chain")
                        })?;
                    if seen[index] {
                        return Err(CuError::from("Duplicate joint in SRDF state"));
                    }
                    let value = number(attr(joint, "value")?)?;
                    positions[index] = positions[index].with_si_value(value);
                    if !self.chain.joints()[index].limits.contains(positions[index]) {
                        return Err(CuError::from("SRDF state violates joint limits"));
                    }
                    seen[index] = true;
                }
                if seen.iter().any(|v| !v) {
                    return Err(CuError::from(
                        "SRDF named state must specify every movable joint",
                    ));
                }
                if states.insert(name.to_owned(), positions).is_some() {
                    return Err(CuError::from("Duplicate SRDF named state"));
                }
            } else if node.has_tag_name("passive_joint")
                && self
                    .joint_names
                    .iter()
                    .any(|n| Some(n.as_str()) == node.attribute("name"))
            {
                return Err(CuError::from(
                    "Passive joints in selected chain are unsupported",
                ));
            }
        }
        self.collision_model = collision_model;
        self.named_states = states;
        Ok(())
    }

    /// Copies a startup SRDF state into caller-provided typed joint storage.
    pub fn write_named_state(&self, name: &str, out: &mut [CuJointPosition; N]) -> CuResult<()> {
        *out = *self
            .named_states
            .get(name)
            .ok_or_else(|| CuError::from("Unknown SRDF named state"))?;
        Ok(())
    }
}

fn robot_root<'a>(doc: &'a Document<'_>) -> CuResult<Node<'a, 'a>> {
    let root = doc.root_element();
    if !root.has_tag_name("robot") {
        return Err(CuError::from("Expected robot XML root"));
    }
    Ok(root)
}
fn attr<'a>(node: Node<'a, '_>, key: &str) -> CuResult<&'a str> {
    node.attribute(key)
        .filter(|v| !v.is_empty())
        .ok_or_else(|| CuError::from(format!("Missing XML attribute {key}")))
}
fn optional_child<'a>(node: Node<'a, 'a>, tag: &str) -> CuResult<Option<Node<'a, 'a>>> {
    let mut found = node.children().filter(|n| n.has_tag_name(tag));
    let first = found.next();
    if found.next().is_some() {
        return Err(CuError::from(format!("Duplicate XML element {tag}")));
    }
    Ok(first)
}
fn child<'a>(node: Node<'a, 'a>, tag: &str) -> CuResult<Node<'a, 'a>> {
    optional_child(node, tag)?.ok_or_else(|| CuError::from(format!("Missing XML element {tag}")))
}
fn named_child<'a>(node: Node<'a, 'a>, tag: &str, name: &str) -> CuResult<Node<'a, 'a>> {
    let mut found = node
        .children()
        .filter(|n| n.has_tag_name(tag) && n.attribute("name") == Some(name));
    let first = found
        .next()
        .ok_or_else(|| CuError::from("Unknown SRDF group"))?;
    if found.next().is_some() {
        return Err(CuError::from("Duplicate SRDF group"));
    }
    Ok(first)
}
fn serial_group<'a>(node: Node<'a, 'a>) -> CuResult<Node<'a, 'a>> {
    let chain = child(node, "chain")?;
    if node.children().filter(Node::is_element).count() != 1 {
        return Err(CuError::from(
            "Only a single serial chain per SRDF group is supported",
        ));
    }
    Ok(chain)
}
fn number(text: &str) -> CuResult<f64> {
    let value: f64 = text
        .parse()
        .map_err(|_| CuError::from("Invalid numeric XML attribute"))?;
    if !value.is_finite() {
        return Err(CuError::from("Non-finite numeric XML attribute"));
    }
    Ok(value)
}
fn positive(text: &str) -> CuResult<f64> {
    let value = number(text)?;
    if value <= 0.0 {
        return Err(CuError::from("Geometry dimensions must be positive"));
    }
    Ok(value)
}
fn vector(text: &str) -> CuResult<[f64; 3]> {
    let mut parts = text.split_whitespace();
    let mut out = [0.0; 3];
    for v in &mut out {
        *v = number(
            parts
                .next()
                .ok_or_else(|| CuError::from("Expected three vector components"))?,
        )?;
    }
    if parts.next().is_some() {
        return Err(CuError::from("Expected three vector components"));
    }
    Ok(out)
}
fn origin(node: Node<'_, '_>) -> CuResult<Transform3D<f64>> {
    let Some(origin) = optional_child(node, "origin")? else {
        return Ok(Transform3D::<f64>::identity());
    };
    let xyz = origin
        .attribute("xyz")
        .map(vector)
        .transpose()?
        .unwrap_or([0.0; 3]);
    let rpy = origin
        .attribute("rpy")
        .map(vector)
        .transpose()?
        .unwrap_or([0.0; 3]);
    Ok(Transform3D::<f64>::from_translation_euler_xyz(
        xyz.map(Length::new::<meter>),
        rpy.map(Angle::new::<radian>),
    ))
}
fn point(m: &[[f64; 4]; 4], p: [f64; 3]) -> [Length; 3] {
    core::array::from_fn(|r| {
        Length::new::<meter>(m[r][0] * p[0] + m[r][1] * p[1] + m[r][2] * p[2] + m[r][3])
    })
}
fn decompose(transform: Transform3D<f64>) -> ([f64; 3], [f64; 3]) {
    let m = transform.to_matrix();
    let pitch = (-m[2][0]).clamp(-1.0, 1.0).asin();
    let (roll, yaw) = if pitch.cos().abs() > 1e-10 {
        (m[2][1].atan2(m[2][2]), m[1][0].atan2(m[0][0]))
    } else {
        (0.0, (-m[0][1]).atan2(m[1][1]))
    };
    ([m[0][3], m[1][3], m[2][3]], [roll, pitch, yaw])
}
