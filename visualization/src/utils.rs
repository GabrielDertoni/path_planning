use wasm_bindgen::prelude::*;

#[macro_export]
macro_rules! console_log {
    // Note that this is using the `log` function imported above during
    // `bare_bones`
    ($($t:tt)*) => (web_sys::console::log_1(&format_args!($($t)*).to_string().into()))
}

#[wasm_bindgen]
extern "C" {
    pub(crate) fn alert(s: &str);
}