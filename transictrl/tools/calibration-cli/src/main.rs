use std::{fs, io::Write, path::PathBuf, process::Command, time::Duration};

use clap::{Parser, Subcommand};
use transictrl_722::framing;

#[derive(Parser)]
#[command(author, version, about = "TransiCtrl calibration CLI", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    New { id: String, name: String, #[arg(short, long)] output: PathBuf },
    Validate { #[arg(short, long)] input: PathBuf },
    ToJson { #[arg(short, long)] input: PathBuf },
    ToToml { #[arg(short, long)] input: PathBuf },
    Import { #[arg(short, long)] converter: PathBuf, #[arg(short, long)] input: PathBuf },
    /// Write a default 722.6 runtime configuration to a file.
    ConfigDefault {
        #[arg(short, long)] output: PathBuf,
        #[arg(long)] json: bool,
    },
    /// Validate a 722.6 runtime configuration.
    ConfigValidate { #[arg(short, long)] input: PathBuf },
    /// Pack a 722.6 config (and optional adaption TOML) into a device blob.
    BlobPack {
        #[arg(short, long)] config: PathBuf,
        #[arg(short, long)] adaption: Option<PathBuf>,
        #[arg(short, long)] output: PathBuf,
    },
    /// Unpack a device blob back into human-readable files.
    BlobUnpack {
        #[arg(short, long)] input: PathBuf,
        #[arg(short, long)] output_dir: PathBuf,
    },
    /// Push a packed blob to a device over a serial port.
    DevicePush {
        #[arg(short, long)] port: String,
        #[arg(short, long)] input: PathBuf,
    },
    /// Pull a packed blob from a device over a serial port.
    DevicePull {
        #[arg(short, long)] port: String,
        #[arg(short, long)] output: PathBuf,
    },
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Commands::New { id, name, output } => {
            let cal = protocol::calibration::CalibrationFile::new_minimal(id, name);
            let toml = toml::to_string_pretty(&cal)?;
            fs::write(output, toml)?;
        }
        Commands::Validate { input } => {
            let s = fs::read_to_string(input)?;
            let cal: protocol::calibration::CalibrationFile = toml::from_str(&s)
                .or_else(|_| serde_json::from_str(&s))?;
            if let Err(e) = cal.validate() {
                anyhow::bail!("invalid calibration: {:?}", e);
            }
            println!("ok: {} {}", cal.metadata.id, cal.metadata.name);
        }
        Commands::ToJson { input } => {
            let s = fs::read_to_string(input)?;
            let cal: protocol::calibration::CalibrationFile = toml::from_str(&s)?;
            println!("{}", serde_json::to_string_pretty(&cal)?);
        }
        Commands::ToToml { input } => {
            let s = fs::read_to_string(input)?;
            let cal: protocol::calibration::CalibrationFile = serde_json::from_str(&s)?;
            println!("{}", toml::to_string_pretty(&cal)?);
        }
        Commands::Import { converter, input } => {
            // Clean-room import: call user-supplied converter, read JSON on stdout
            let out = Command::new(converter)
                .arg(input)
                .output()?;
            if !out.status.success() {
                anyhow::bail!("converter failed: status {:?}", out.status);
            }
            let json = String::from_utf8(out.stdout)?;
            let cal: protocol::calibration::CalibrationFile = serde_json::from_str(&json)?;
            println!("{}", toml::to_string_pretty(&cal)?);
        }
        Commands::ConfigDefault { output, json } => {
            let cfg = transictrl_722::Config722::default();
            let s = if json {
                serde_json::to_string_pretty(&cfg)?
            } else {
                toml::to_string_pretty(&cfg)?
            };
            fs::write(output, s)?;
        }
        Commands::ConfigValidate { input } => {
            let s = fs::read_to_string(input)?;
            let cfg: transictrl_722::Config722 = toml::from_str(&s)
                .or_else(|_| serde_json::from_str(&s))?;
            cfg.validate()
                .map_err(|e| anyhow::anyhow!("invalid config: {e}"))?;
            println!("ok: 722.6 config validates");
        }
        Commands::BlobPack { config, adaption, output } => {
            let s = fs::read_to_string(config)?;
            let cfg: transictrl_722::Config722 = toml::from_str(&s)
                .or_else(|_| serde_json::from_str(&s))?;
            cfg.validate()
                .map_err(|e| anyhow::anyhow!("invalid config: {e}"))?;

            let mut blob = transictrl_722::persist::CalibrationBlob {
                config: cfg,
                ..Default::default()
            };
            if let Some(path) = adaption {
                let s = fs::read_to_string(path)?;
                blob.adaption = toml::from_str(&s)
                    .or_else(|_| serde_json::from_str(&s))?;
            }

            let mut buf = [0u8; 512];
            let bytes = postcard::to_slice(&blob, &mut buf)?;
            fs::write(output, bytes)?;
        }
        Commands::BlobUnpack { input, output_dir } => {
            let bytes = fs::read(input)?;
            let blob: transictrl_722::persist::CalibrationBlob = postcard::from_bytes(&bytes)?;
            fs::create_dir_all(&output_dir)?;
            let config_toml = toml::to_string_pretty(&blob.config)
                .map_err(|e| anyhow::anyhow!("config toml: {e}"))?;
            fs::write(output_dir.join("config.toml"), config_toml)?;
            let adaption_json = serde_json::to_string_pretty(&blob.adaption)
                .map_err(|e| anyhow::anyhow!("adaption json: {e}"))?;
            fs::write(output_dir.join("adaption.json"), adaption_json)?;
            println!("unpacked to {}", output_dir.display());
        }
        Commands::DevicePush { port, input } => {
            let bytes = fs::read(input)?;
            if bytes.len() > u16::MAX as usize {
                anyhow::bail!("blob too large");
            }
            let mut port = serialport::new(port, 115_200)
                .timeout(Duration::from_secs(2))
                .open()
                .map_err(|e| anyhow::anyhow!("serial open: {e}"))?;
            let mut frame = vec![0u8; bytes.len() + framing::OVERHEAD];
            framing::encode(framing::host_op::PUSH_BLOB, &bytes, &mut frame);
            port.write_all(&frame)?;
            port.flush()?;
            wait_ack(&mut port)?;
            println!("pushed {} bytes", bytes.len());
        }
        Commands::DevicePull { port, output } => {
            let mut port = serialport::new(port, 115_200)
                .timeout(Duration::from_secs(2))
                .open()
                .map_err(|e| anyhow::anyhow!("serial open: {e}"))?;
            let mut frame = [0u8; framing::OVERHEAD];
            framing::encode(framing::host_op::PULL_BLOB, &[], &mut frame);
            port.write_all(&frame)?;
            port.flush()?;
            let blob = recv_blob(&mut port)?;
            fs::write(output, &blob)?;
            println!("pulled {} bytes", blob.len());
        }
    }
    Ok(())
}

fn wait_ack(port: &mut Box<dyn serialport::SerialPort>) -> anyhow::Result<()> {
    let deadline = std::time::Instant::now() + Duration::from_secs(3);
    let mut buf = [0u8; 256];
    let mut accumulated = Vec::new();
    while std::time::Instant::now() < deadline {
        match port.read(&mut buf) {
            Ok(0) => {}
            Ok(n) => {
                accumulated.extend_from_slice(&buf[..n]);
                if let Some(pos) = framing::find_sync(&accumulated) {
                    let rest = &accumulated[pos..];
                    if rest.len() >= framing::OVERHEAD {
                        let mut payload = [0u8; 8];
                        if let Ok((op, _)) = framing::decode(rest, &mut payload) {
                            if op == framing::device_op::ACK {
                                return Ok(());
                            }
                            anyhow::bail!("device returned opcode 0x{op:02x}, expected ACK");
                        }
                    }
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => return Err(anyhow::anyhow!("serial read: {e}")),
        }
    }
    anyhow::bail!("timed out waiting for device ACK")
}

fn recv_blob(port: &mut Box<dyn serialport::SerialPort>) -> anyhow::Result<Vec<u8>> {
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    let mut buf = [0u8; 256];
    let mut accumulated = Vec::new();
    while std::time::Instant::now() < deadline {
        match port.read(&mut buf) {
            Ok(0) => {}
            Ok(n) => {
                accumulated.extend_from_slice(&buf[..n]);
                if let Some(pos) = framing::find_sync(&accumulated) {
                    let rest = &accumulated[pos..];
                    // Need at least the header to know the payload length.
                    if rest.len() >= framing::HEADER_LEN {
                        let payload_len =
                            ((rest[2] as usize) << 8) | rest[3] as usize;
                        if rest.len() >= payload_len + framing::OVERHEAD {
                            let mut payload = vec![0u8; payload_len];
                            if let Ok((op, len)) = framing::decode(rest, &mut payload) {
                                if op == framing::device_op::BLOB {
                                    payload.truncate(len);
                                    return Ok(payload);
                                }
                                anyhow::bail!("device returned opcode 0x{op:02x}, expected BLOB");
                            }
                        }
                    }
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => return Err(anyhow::anyhow!("serial read: {e}")),
        }
    }
    anyhow::bail!("timed out waiting for device blob")
}
