fn main() {
    #[cfg(windows)]
    {
        let mut res = winres::WindowsResource::new();
        res.set_icon("assets/icon.ico");
        res.set("ProductName", "ConeFlattener 3000");
        res.set("FileDescription", "Formula Student Drone-to-Birdseye Track Map Rectifier");
        res.set("LegalCopyright", "Formula Student");
        res.compile().unwrap();
    }
}
