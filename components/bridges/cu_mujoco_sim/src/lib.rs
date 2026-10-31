//! [`CuMujocoSimBridge`]: a [`CuBridge`](cu29::prelude::CuBridge) connecting Copper to an
//! external MuJoCo simulator sidecar process over TCP.
//!
//! This crate is `std`-only: bridges are host components and this one owns a
//! blocking [`TcpStream`](std::net::TcpStream). There is no `no_std` surface.
//!
//! # Wire protocol
//!
//! TCP, line-delimited JSON, one request/response per Copper cycle. The sidecar
//! listens; the bridge is the TCP client.
//!
//! Bridge send (Tx), joint position setpoints in radians (SI values):
//!
//! ```text
//! {"type":"command","q":[q0,q1,q2,q3,q4,q5]}\n
//! ```
//!
//! Bridge receive (Rx), after the sidecar steps physics:
//!
//! ```text
//! {"type":"state","q":[q0..q5],"qd":[qd0..qd5],"time":<sim seconds>}\n
//! ```
//!
//! Receive parses `q` into positions and `qd` into velocities. `time` is
//! ignored. A malformed line (or a missing/non-numeric `q`/`qd` array) is a
//! [`CuError`](cu29::prelude::CuError).
//!
//! # Payload contract
//!
//! The Tx `joint_command` channel carries
//! [`CuJointCommandPayload<6>`](cu_controller::CuJointCommandPayload) and the
//! Rx `joint_state` channel carries
//! [`CuJointStatePayload<6>`](cu_kinematics::CuJointStatePayload): the
//! unified controller contract for the 6-DOF igus ReBeL arm. `N` is fixed at
//! 6 in v1; the sidecar is built to match. Joints are revolute, positions
//! are SI radians, velocities SI radians per second. Only the command
//! positions reach the wire; command velocities/status stay on the Copper
//! side.
//!
//! # Config
//!
//! The bridge `config:` block is a [`CuMujocoSimParams`]; unknown keys are
//! rejected.
//!
//! | Key | Type | Default | Meaning |
//! |---|---|---|---|
//! | `host` | `String` | `"127.0.0.1"` | sidecar address |
//! | `port` | `u16` | `8765` | sidecar TCP port |
//! | `read_timeout` | `Time` (s) | `0.25` | socket read (and connect/write) timeout |
//!
//! The bridge connects in
//! [`start`](cu29::prelude::CuBridge::start) and reconnects lazily in
//! `send`/`receive` when the stream is dead. Reads always carry a timeout, so
//! the runtime can never hang forever on a silent sidecar.
//!
//! # `copperconfig.ron` sketch
//!
//! ```ron
//! bridges: [
//!     (
//!         id: "mujoco",
//!         type: "bridges::CuMujocoSimBridge",
//!         config: {
//!             "host": "127.0.0.1",
//!             "port": 8765,
//!             "read_timeout": 0.25,
//!         },
//!         channels: [
//!             Tx(id: "joint_command"),
//!             Rx(id: "joint_state"),
//!         ],
//!     ),
//! ],
//! cnx: [
//!     ( src: "controller", dst: "mujoco/joint_command",
//!       msg: "cu_controller::CuJointCommandPayload<6>" ),
//!     ( src: "mujoco/joint_state", dst: "controller",
//!       msg: "cu_kinematics::CuJointStatePayload<6>" ),
//! ],
//! ```
//!
//! Channel routes default to `"joint_command"` (Tx) and `"joint_state"` (Rx).

use bincode::Decode;
use bincode::Encode;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::DecodeError;
use bincode::error::EncodeError;
use cu_controller::CuJointCommandPayload;
use cu_controller::CuJointEffort;
use cu_kinematics::CuJointPosition;
use cu_kinematics::CuJointStatePayload;
use cu_kinematics::CuJointVelocity;
use cu_kinematics::CuRobotStatus;
use cu29::prelude::*;
use cu29::rx_channels;
use cu29::tx_channels;
use cu29::units::si::angle::radian;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::AngularVelocity;
use cu29::units::si::f64::Time;
use cu29::units::si::time::second;
use serde::Deserialize;
use serde::Serialize;
use std::io::Read;
use std::io::Write;
use std::net::TcpStream;
use std::net::ToSocketAddrs;
use std::time::Duration;

/// Parameters of [`CuMujocoSimBridge`]; the bridge `config:` block.
///
/// ```rust
/// use cu_mujoco_sim::CuMujocoSimParams;
/// use cu29::units::si::f64::Time;
/// use cu29::units::si::time::second;
///
/// let params = CuMujocoSimParams {
///     port: 9000,
///     ..CuMujocoSimParams::default()
/// };
/// assert_eq!(params.host, "127.0.0.1");
/// assert_eq!(params.read_timeout, Time::new::<second>(0.25));
/// params.validate().unwrap();
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuMujocoSimParams {
    /// Sidecar host name or address.
    pub host: String,
    /// Sidecar TCP port.
    pub port: u16,
    /// Socket read (and connect/write) timeout; positive and finite.
    pub read_timeout: Time,
}

impl Default for CuMujocoSimParams {
    /// `127.0.0.1:8765`, 0.25 s timeout.
    fn default() -> Self {
        Self {
            host: "127.0.0.1".into(),
            port: 8765,
            read_timeout: Time::new::<second>(0.25),
        }
    }
}

impl CuMujocoSimParams {
    /// Checks that the host is non-empty and the timeout is positive, finite,
    /// and representable as a [`Duration`].
    pub fn validate(&self) -> CuResult<()> {
        if self.host.is_empty() {
            return Err(CuError::from("cu-mujoco-sim: 'host' must not be empty"));
        }
        Duration::try_from_secs_f64(self.read_timeout.get::<second>())
            .ok()
            .filter(|d| !d.is_zero())
            .ok_or_else(|| {
                CuError::from("cu-mujoco-sim: 'read_timeout' must be positive and finite")
            })?;
        Ok(())
    }
}
/// Fixed joint count for v1 (igus ReBeL 6-DOF).
pub const MUJOCO_DOF: usize = 6;
/// Largest accepted wire line; bounds the stack read buffer.
const MAX_LINE_BYTES: usize = 1024;

tx_channels! {
    pub struct MujocoTxChannels: MujocoTxId {
        joint_command => CuJointCommandPayload<6> = "joint_command",
    }
}

rx_channels! {
    pub struct MujocoRxChannels: MujocoRxId {
        joint_state => CuJointStatePayload<6> = "joint_state",
    }
}

/// `CuBridge` to an external MuJoCo simulator sidecar over TCP/JSON.
///
/// Owns one TCP session and multiplexes the command/state exchange over it.
/// `freeze`/`thaw` persist the connection config only; the socket itself is
/// re-established on `start`, so keyframe restore never resurrects a stale fd.
#[derive(Reflect)]
pub struct CuMujocoSimBridge {
    #[reflect(ignore)]
    params: CuMujocoSimParams,
    #[reflect(ignore)]
    stream: Option<TcpStream>,
}

impl CuMujocoSimBridge {
    /// Creates a bridge from validated params (the socket connects in `start`).
    pub fn from_params(params: CuMujocoSimParams) -> CuResult<Self> {
        params.validate()?;
        Ok(Self {
            params,
            stream: None,
        })
    }

    /// Params this bridge runs with.
    pub fn params(&self) -> &CuMujocoSimParams {
        &self.params
    }

    fn timeout(&self) -> Duration {
        Duration::from_secs_f64(self.params.read_timeout.get::<second>())
    }

    fn connect(&self) -> CuResult<TcpStream> {
        let addr = (self.params.host.as_str(), self.params.port)
            .to_socket_addrs()
            .map_err(|e| {
                CuError::new_with_cause("cu-mujoco-sim: cannot resolve sidecar address", e)
            })?
            .next()
            .ok_or_else(|| CuError::from("cu-mujoco-sim: sidecar address resolved to nothing"))?;
        let stream = TcpStream::connect_timeout(&addr, self.timeout())
            .map_err(|e| CuError::new_with_cause("cu-mujoco-sim: cannot connect to sidecar", e))?;
        stream
            .set_read_timeout(Some(self.timeout()))
            .map_err(|e| CuError::new_with_cause("cu-mujoco-sim: cannot set read timeout", e))?;
        stream
            .set_write_timeout(Some(self.timeout()))
            .map_err(|e| CuError::new_with_cause("cu-mujoco-sim: cannot set write timeout", e))?;
        Ok(stream)
    }

    fn ensure_connected(&mut self) -> CuResult<&mut TcpStream> {
        match self.stream {
            Some(ref mut stream) => Ok(stream),
            None => {
                let stream = self.connect()?;
                Ok(self.stream.insert(stream))
            }
        }
    }

    fn drop_stream(&mut self) {
        self.stream = None;
    }
}

/// Encodes a joint command payload as one protocol line (with `\n`).
///
/// Only the command positions reach the wire; velocities and status stay on
/// the Copper side.
pub(crate) fn encode_command_line(payload: &CuJointCommandPayload<MUJOCO_DOF>) -> CuResult<String> {
    let q: [f64; MUJOCO_DOF] = payload.positions.map(CuJointPosition::si_value);
    let qd: [f64; MUJOCO_DOF] = payload.velocities.map(CuJointVelocity::si_value);
    let effort: [f64; MUJOCO_DOF] = payload.efforts.map(CuJointEffort::si_value);
    let mut line = serde_json::to_string(
        &serde_json::json!({"type": "command", "q": q, "qd": qd, "effort": effort, "mode": payload.mode}),
    )
    .map_err(|e| CuError::new_with_cause("cu-mujoco-sim: cannot encode command", e))?;
    line.push('\n');
    Ok(line)
}

/// Decodes one sidecar state line into `out`, parsing `q` into positions and
/// `qd` into velocities (`time` is ignored).
///
/// `out` is left untouched when the line is malformed; the error is returned.
pub(crate) fn decode_state_line(
    line: &str,
    out: &mut CuJointStatePayload<MUJOCO_DOF>,
) -> CuResult<()> {
    let value: serde_json::Value = serde_json::from_str(line)
        .map_err(|e| CuError::new_with_cause("cu-mujoco-sim: bad JSON", e))?;
    if value.get("type").and_then(serde_json::Value::as_str) != Some("state") {
        return Err(CuError::from(
            "cu-mujoco-sim: expected {\"type\":\"state\",...}",
        ));
    }
    let q = value
        .get("q")
        .and_then(serde_json::Value::as_array)
        .ok_or_else(|| CuError::from("cu-mujoco-sim: state line misses array \"q\""))?;
    if q.len() != MUJOCO_DOF {
        return Err(CuError::from(
            "cu-mujoco-sim: state \"q\" must hold 6 joints",
        ));
    }
    let qd = value
        .get("qd")
        .and_then(serde_json::Value::as_array)
        .ok_or_else(|| CuError::from("cu-mujoco-sim: state line misses array \"qd\""))?;
    if qd.len() != MUJOCO_DOF {
        return Err(CuError::from(
            "cu-mujoco-sim: state \"qd\" must hold 6 joints",
        ));
    }
    let mut positions = [CuJointPosition::default(); MUJOCO_DOF];
    for (i, v) in q.iter().enumerate() {
        let rad = v
            .as_f64()
            .ok_or_else(|| CuError::from("cu-mujoco-sim: state \"q\" holds non-numeric joint"))?;
        positions[i] = CuJointPosition::Revolute(Angle::new::<radian>(rad));
    }
    let mut velocities = [CuJointVelocity::default(); MUJOCO_DOF];
    for (i, v) in qd.iter().enumerate() {
        let rad_per_s = v
            .as_f64()
            .ok_or_else(|| CuError::from("cu-mujoco-sim: state \"qd\" holds non-numeric joint"))?;
        velocities[i] =
            CuJointVelocity::Revolute(AngularVelocity::new::<radian_per_second>(rad_per_s));
    }
    out.positions = positions;
    out.velocities = velocities;
    out.status = CuRobotStatus::Ready;
    Ok(())
}

impl Freezable for CuMujocoSimBridge {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.params.host, encoder)?;
        Encode::encode(&self.params.port, encoder)?;
        Encode::encode(&self.params.read_timeout.get::<second>(), encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.params.host = Decode::decode(decoder)?;
        self.params.port = Decode::decode(decoder)?;
        let read_timeout: f64 = Decode::decode(decoder)?;
        self.params.read_timeout = Time::new::<second>(read_timeout);
        self.stream = None;
        Ok(())
    }
}

impl CuBridge for CuMujocoSimBridge {
    type Tx = MujocoTxChannels;
    type Rx = MujocoRxChannels;
    type Resources<'r> = ();

    fn new(
        config: Option<&ComponentConfig>,
        _tx_channels: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        _rx_channels: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        let params = config
            .map(ComponentConfig::deserialize_into::<CuMujocoSimParams>)
            .transpose()?
            .unwrap_or_default();
        Self::from_params(params)
    }

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.stream = Some(self.connect()?);
        Ok(())
    }

    fn send<'a, Payload>(
        &mut self,
        _ctx: &CuContext,
        channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        if channel.id != MujocoTxId::JointCommand {
            return Err(CuError::from("cu-mujoco-sim: unknown Tx channel"));
        }
        let cmd = msg.downcast_ref::<CuJointCommandPayload<MUJOCO_DOF>>()?;
        let Some(payload) = cmd.payload() else {
            return Ok(());
        };
        let line = encode_command_line(payload)?;
        let stream = self.ensure_connected()?;
        let result = stream
            .write_all(line.as_bytes())
            .and_then(|()| stream.flush());
        if let Err(e) = result {
            self.drop_stream();
            return Err(CuError::new_with_cause(
                "cu-mujoco-sim: command write failed",
                e,
            ));
        }
        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        if channel.id != MujocoRxId::JointState {
            return Err(CuError::from("cu-mujoco-sim: unknown Rx channel"));
        }
        let stream = self.ensure_connected()?;
        let mut buf = [0u8; MAX_LINE_BYTES];
        let mut len = 0usize;
        loop {
            if len >= buf.len() {
                self.drop_stream();
                return Err(CuError::from("cu-mujoco-sim: state line too long"));
            }
            let n = match stream.read(&mut buf[len..len + 1]) {
                Ok(0) => {
                    self.drop_stream();
                    return Err(CuError::from(
                        "cu-mujoco-sim: sidecar closed the connection",
                    ));
                }
                Ok(n) => n,
                Err(e) => {
                    self.drop_stream();
                    return Err(CuError::new_with_cause(
                        "cu-mujoco-sim: state read failed",
                        e,
                    ));
                }
            };
            len += n;
            if buf[len - 1] == b'\n' {
                break;
            }
        }
        let line = core::str::from_utf8(&buf[..len])
            .map_err(|e| CuError::new_with_cause("cu-mujoco-sim: state line is not UTF-8", e))?;
        let state = msg.downcast_mut::<CuJointStatePayload<MUJOCO_DOF>>()?;
        state.tov = Tov::Time(ctx.now());
        let Some(payload) = state.payload_mut() else {
            let mut fresh = CuJointStatePayload::<MUJOCO_DOF>::default();
            decode_state_line(line, &mut fresh)?;
            state.set_payload(fresh);
            return Ok(());
        };
        decode_state_line(line, payload)
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.drop_stream();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu_controller::CuControllerMode;
    use std::net::TcpListener;
    use std::sync::mpsc::channel;
    use std::thread::spawn;

    fn test_command(q: [f64; MUJOCO_DOF]) -> CuJointCommandPayload<MUJOCO_DOF> {
        CuJointCommandPayload {
            positions: q.map(|v| CuJointPosition::Revolute(Angle::new::<radian>(v))),
            ..Default::default()
        }
    }

    #[test]
    fn test_encode_command_line_shape() {
        let line = encode_command_line(&test_command([0.0, 0.5, -1.0, 1.5, -0.25, 3.0])).unwrap();
        assert!(line.ends_with('\n'));
        let value: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(value.get("type").and_then(|t| t.as_str()), Some("command"));
        let q = value.get("q").and_then(|q| q.as_array()).unwrap();
        assert_eq!(q.len(), MUJOCO_DOF);
        for (got, want) in q.iter().zip([0.0, 0.5, -1.0, 1.5, -0.25, 3.0]) {
            assert!((got.as_f64().unwrap() - want).abs() < 1e-12);
        }
        // Default command is position mode with zero velocity/effort.
        assert_eq!(value.get("mode").and_then(|m| m.as_str()), Some("position"));
        assert_eq!(
            value.get("qd").and_then(|v| v.as_array()).unwrap().len(),
            MUJOCO_DOF
        );
        assert_eq!(
            value
                .get("effort")
                .and_then(|v| v.as_array())
                .unwrap()
                .len(),
            MUJOCO_DOF
        );
    }

    #[test]
    fn test_encode_command_line_carries_effort_mode() {
        let mut payload = test_command([0.0; MUJOCO_DOF]);
        payload.mode = CuControllerMode::Effort;
        payload.efforts = [CuJointEffort::Revolute(cu29::units::si::f64::Torque::new::<
            cu29::units::si::torque::newton_meter,
        >(2.5)); MUJOCO_DOF];
        let line = encode_command_line(&payload).unwrap();
        let value: serde_json::Value = serde_json::from_str(&line).unwrap();
        assert_eq!(value.get("mode").and_then(|m| m.as_str()), Some("effort"));
        let effort = value.get("effort").and_then(|v| v.as_array()).unwrap();
        assert!((effort[0].as_f64().unwrap() - 2.5).abs() < 1e-12);
    }

    #[test]
    fn test_decode_state_line_round_trip() {
        let line = "{\"type\":\"state\",\"q\":[0.1,0.2,0.3,0.4,0.5,0.6],\"qd\":[0.01,0.02,0.03,0.04,0.05,0.06],\"time\":1.25}\n";
        let mut out = CuJointStatePayload::<MUJOCO_DOF>::default();
        decode_state_line(line, &mut out).unwrap();
        for (i, want) in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6].iter().enumerate() {
            assert!((out.positions[i].si_value() - want).abs() < 1e-12);
        }
        for (i, want) in [0.01, 0.02, 0.03, 0.04, 0.05, 0.06].iter().enumerate() {
            assert!((out.velocities[i].si_value() - want).abs() < 1e-12);
        }
    }

    #[test]
    fn test_decode_state_line_rejects_malformed() {
        let mut out = CuJointStatePayload::<MUJOCO_DOF>::default();
        for bad in [
            "not json\n",
            "{\"type\":\"command\",\"q\":[0,0,0,0,0,0]}\n",
            "{\"type\":\"state\",\"qd\":[0,0,0,0,0,0]}\n",
            "{\"type\":\"state\",\"q\":[0,0,0,0,0]}\n",
            "{\"type\":\"state\",\"q\":[0,0,0,0,0,0,0]}\n",
            "{\"type\":\"state\",\"q\":[0,0,0,0,0,\"x\"],\"qd\":[0,0,0,0,0,0]}\n",
            "{\"type\":\"state\",\"q\":[0,0,0,0,0,0]}\n",
            "{\"type\":\"state\",\"q\":[0,0,0,0,0,0],\"qd\":[0,0,0,0,0]}\n",
            "{\"type\":\"state\",\"q\":[0,0,0,0,0,0],\"qd\":[0,0,0,0,0,\"x\"]}\n",
        ] {
            assert!(decode_state_line(bad, &mut out).is_err(), "accepted: {bad}");
        }
        // Rejected lines leave the payload untouched (still default zeros).
        for p in out.positions {
            assert_eq!(p.si_value(), 0.0);
        }
        for v in out.velocities {
            assert_eq!(v.si_value(), 0.0);
        }
    }

    #[test]
    fn test_new_defaults() {
        let bridge = CuMujocoSimBridge::new(None, &[], &[], ()).unwrap();
        assert_eq!(bridge.params().host, "127.0.0.1");
        assert_eq!(bridge.params().port, 8765);
        assert_eq!(bridge.timeout(), Duration::from_millis(250));
    }

    #[test]
    fn test_new_reads_config() {
        let cfg: ComponentConfig = ron::from_str(
            r#"#![enable(unwrap_newtypes)] {"host": "10.0.0.9", "port": 9999, "read_timeout": 0.042}"#,
        )
        .unwrap();
        let bridge = CuMujocoSimBridge::new(Some(&cfg), &[], &[], ()).unwrap();
        assert_eq!(bridge.params().host, "10.0.0.9");
        assert_eq!(bridge.params().port, 9999);
        assert_eq!(bridge.timeout(), Duration::from_millis(42));
        for bad in [
            r#"{"read_timeout_ms": 250}"#,
            r#"{"read_timeout": 0.0}"#,
            r#"{"host": ""}"#,
        ] {
            let cfg: ComponentConfig =
                ron::from_str(&format!("#![enable(unwrap_newtypes)] {bad}")).unwrap();
            assert!(
                CuMujocoSimBridge::new(Some(&cfg), &[], &[], ()).is_err(),
                "{bad}"
            );
        }
    }

    #[test]
    fn test_freeze_thaw_restores_config_without_socket() {
        let mut bridge = CuMujocoSimBridge::from_params(CuMujocoSimParams {
            host: "10.0.0.9".into(),
            port: 9999,
            read_timeout: Time::new::<second>(0.042),
        })
        .unwrap();
        let mut bytes = [0u8; 256];
        let mut encoder = bincode::enc::EncoderImpl::new(
            bincode::enc::write::SliceWriter::new(&mut bytes),
            bincode::config::standard(),
        );
        bridge.freeze(&mut encoder).unwrap();
        let len = encoder.into_writer().bytes_written();
        bridge.params.host = "changed".into();
        let mut decoder = bincode::de::DecoderImpl::new(
            bincode::de::read::SliceReader::new(&bytes[..len]),
            bincode::config::standard(),
            (),
        );
        bridge.thaw(&mut decoder).unwrap();
        assert_eq!(bridge.params().host, "10.0.0.9");
        assert_eq!(bridge.params().port, 9999);
        assert_eq!(bridge.timeout(), Duration::from_millis(42));
    }

    /// Loopback sidecar: one command line in, one state line out.
    /// Binds port 0 (ephemeral) and hands the port over a channel, so the
    /// test cannot collide with a fixed port.
    #[test]
    fn test_loopback_command_state_cycle() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let (ready_tx, ready_rx) = channel();
        let sidecar = spawn(move || {
            ready_tx.send(()).unwrap();
            let (mut sock, _) = listener.accept().unwrap();
            sock.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
            let mut buf = [0u8; MAX_LINE_BYTES];
            let mut len = 0usize;
            while !buf[..len].contains(&b'\n') {
                len += sock.read(&mut buf[len..]).unwrap();
            }
            let value: serde_json::Value =
                serde_json::from_str(core::str::from_utf8(&buf[..len]).unwrap()).unwrap();
            assert_eq!(value.get("type").and_then(|t| t.as_str()), Some("command"));
            let reply = "{\"type\":\"state\",\"q\":[0.1,0.2,0.3,0.4,0.5,0.6],\"qd\":[0,0,0,0,0,0],\"time\":0.01}\n";
            sock.write_all(reply.as_bytes()).unwrap();
            sock.flush().unwrap();
        });
        ready_rx.recv().unwrap();

        let (ctx, _) = CuContext::new_mock_clock();
        let mut bridge = CuMujocoSimBridge::from_params(CuMujocoSimParams {
            port,
            read_timeout: Time::new::<second>(5.0),
            ..CuMujocoSimParams::default()
        })
        .unwrap();
        bridge.start(&ctx).unwrap();
        let cmd = CuMsg::new(Some(test_command([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])));
        bridge
            .send(&ctx, &MujocoTxChannels::JOINT_COMMAND, &cmd)
            .unwrap();
        let mut state = CuMsg::<CuJointStatePayload<MUJOCO_DOF>>::new(None);
        bridge
            .receive(&ctx, &MujocoRxChannels::JOINT_STATE, &mut state)
            .unwrap();
        let payload = state.payload().unwrap();
        for (i, want) in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6].iter().enumerate() {
            assert!((payload.positions[i].si_value() - want).abs() < 1e-12);
        }
        bridge.stop(&ctx).unwrap();
        sidecar.join().unwrap();
    }
}
