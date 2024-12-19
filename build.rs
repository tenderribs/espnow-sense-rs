use chrono::Utc;

fn main() {
    println!("cargo:rustc-link-arg-bins=-Tlinkall.x");

    // pass the current UTC time
    println!("cargo::rustc-env=BUILD_TIME={}", Utc::now());
}
