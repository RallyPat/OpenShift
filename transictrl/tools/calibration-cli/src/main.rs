use std::{fs, path::PathBuf, process::Command};

use clap::{Parser, Subcommand};

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
    }
    Ok(())
}
