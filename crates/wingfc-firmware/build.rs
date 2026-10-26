use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");

    // Watch root aircraft configuration and builder script
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let yaml_path = manifest_dir.join("../../wingfc.yaml");
    let script_path = manifest_dir.join("../../scripts/builder.py");
    let target_config = manifest_dir.join("src/config.rs");

    println!("cargo:rerun-if-changed={}", yaml_path.display());
    println!("cargo:rerun-if-changed={}", script_path.display());

    if yaml_path.exists() {
        let status = Command::new("python3")
            .arg(&script_path)
            .arg("generate")
            .arg("--out-config")
            .arg(&target_config)
            .status();
        match status {
            Ok(s) if s.success() => {}
            Ok(s) => panic!(
                "wingfc.yaml validation or config generation failed with exit code: {:?}",
                s.code()
            ),
            Err(e) => eprintln!(
                "cargo:warning=Failed to execute python3 {}: {}",
                script_path.display(),
                e
            ),
        }

        // Check if active Cargo receiver feature matches wingfc.yaml
        if let Ok(content) = std::fs::read_to_string(&yaml_path) {
            let is_ibus_yaml = content.contains("protocol: ibus")
                || content.contains("protocol: \"ibus\"")
                || content.contains("protocol: 'ibus'");
            let is_crsf_yaml = content.contains("protocol: crsf")
                || content.contains("protocol: \"crsf\"")
                || content.contains("protocol: 'crsf'");
            let crsf_feat = env::var("CARGO_FEATURE_CRSF").is_ok();
            let ibus_feat = env::var("CARGO_FEATURE_IBUS").is_ok();

            if is_ibus_yaml && crsf_feat && !ibus_feat {
                println!(
                    "cargo:warning=wingfc.yaml specifies protocol 'ibus', but building with feature 'crsf' (420k baud). To build ibus, use: cargo run --no-default-features --features ibus"
                );
            }
            if is_crsf_yaml && ibus_feat && !crsf_feat {
                println!(
                    "cargo:warning=wingfc.yaml specifies protocol 'crsf', but building with feature 'ibus' (115.2k baud). To build crsf, use: cargo run --no-default-features --features crsf"
                );
            }
        }
    }
}
