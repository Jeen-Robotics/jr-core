# Rust middleware crate integration via Corrosion
corrosion_import_crate(MANIFEST_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml)

# Export the middleware_rs target for linking
# The target name matches the crate name in Cargo.toml
