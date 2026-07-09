use crate::gui::SotaRoutingApp;
use crate::Point;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct WebHandle {
    runner: eframe::WebRunner,
}

#[wasm_bindgen]
impl WebHandle {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        console_error_panic_hook::set_once();
        Self {
            runner: eframe::WebRunner::new(),
        }
    }

    #[wasm_bindgen]
    pub async fn start(
        &self,
        canvas: web_sys::HtmlCanvasElement,
        start_lat: f64,
        start_lon: f64,
        dest_lat: f64,
        dest_lon: f64,
    ) -> Result<(), wasm_bindgen::JsValue> {
        let start = Point::new(start_lat, start_lon);
        let dest = Some(Point::new(dest_lat, dest_lon));
        self.runner
            .start(
                canvas,
                eframe::WebOptions::default(),
                Box::new(move |_cc| Ok(Box::new(SotaRoutingApp::new(start, dest)))),
            )
            .await
    }

    #[wasm_bindgen]
    pub fn destroy(&self) {
        self.runner.destroy();
    }
}
