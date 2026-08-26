fn main() {
    println!("cargo:rerun-if-changed=assets/icon.ico");
    println!("cargo:rerun-if-changed=assets/app_logo.jpg");
    #[cfg(windows)]
    {
        let mut res = winres::WindowsResource::new();
        res.set_icon("assets/icon.ico");
        res.set("ProductName", "HomoKuŽel");
        res.set("FileDescription", "Formula Student Drone-to-Birdseye Track Map Rectifier");
        res.set("LegalCopyright", "Formula Student");
        res.compile().unwrap();
    }
}
