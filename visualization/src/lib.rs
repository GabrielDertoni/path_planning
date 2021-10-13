#[macro_use]
mod utils;

use nalgebra as na;
use plotters::prelude::*;
use plotters_canvas::CanvasBackend;
use std::borrow::Borrow;
use std::collections::HashSet;
use wasm_bindgen::prelude::*;
use web_sys::HtmlCanvasElement;

use kd_tree::HasCoords;
use rrt::{
    obstacle::{self as obs, Obstacle},
    solvers::star::RRTStarIter,
    RRTBuilder,
};

// use utils::*;

// When the `wee_alloc` feature is enabled, this uses `wee_alloc` as the global
// allocator.
//
// If you don't want to use `wee_alloc`, you can safely delete this.
#[cfg(feature = "wee_alloc")]
#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

#[wasm_bindgen]
pub fn init() {
    #[cfg(debug_assertions)]
    console_error_panic_hook::set_once();
}

fn point2(x: f32, y: f32) -> na::Point2<f32> {
    na::Point2::new(x, y)
}

fn vec2(x: f32, y: f32) -> na::Vector2<f32> {
    na::Vector2::from([x, y])
}

#[wasm_bindgen]
pub struct RRTAlgorithm {
    target: na::Point<f32, 2>,
    origin: na::Point<f32, 2>,
    obstacles: Vec<obs::Rectangle<2>>,
    iter: RRTStarIter<Box<dyn Obstacle<2>>, 2>,
    has_ended: bool,
}

type Error = Box<dyn std::error::Error>;

#[wasm_bindgen]
impl RRTAlgorithm {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        let target = point2(5.0, 5.0);
        let origin = point2(65.0, 65.0);

        let obstacles = &[
            obs::Rectangle::new(point2(10.0, 10.0), vec2(50.0, 50.0)),
            obs::Rectangle::new(point2(70.0, 10.0), vec2(50.0, 50.0)),
            obs::Rectangle::new(point2(10.0, 70.0), vec2(50.0, 50.0)),
            obs::Rectangle::new(point2(70.0, 70.0), vec2(50.0, 50.0)),
        ];

        let builder = RRTBuilder::new(origin, target)
            .with_dyn_obstacles()
            .extend_obstacles(
                obstacles
                    .iter()
                    .cloned()
                    .map(|el| -> Box<dyn Obstacle<2>> { Box::new(el) }),
            )
            .with_step_size(1.0)
            .with_target_radius(1.0)
            .with_max_stepping(1)
            .with_update_radius(1.0)
            .with_random_range_start(point2(0.0, 0.0))
            .with_random_range_end(point2(130.0, 130.0));

        RRTAlgorithm {
            target,
            origin,
            obstacles: obstacles.iter().cloned().collect(),
            iter: RRTStarIter::from_builder(builder),
            has_ended: false,
        }
    }

    pub fn iters(&self) -> usize {
        self.iter.iters
    }

    pub fn step(&mut self) {
        if !self.has_ended {
            self.has_ended = self.iter.next().is_none();
        }
    }

    pub fn draw(&mut self, canvas: HtmlCanvasElement) -> Result<(), JsValue> {
        self.try_draw(canvas).map_err(|err| err.to_string())?;
        Ok(())
    }

    fn try_draw(&self, canvas: HtmlCanvasElement) -> Result<(), Error> {
        let soft_red = RGBColor(200, 50, 50);

        let &RRTStarIter {
            ref from,
            reached_idx,
            ref kd_tree,
            ..
        } = &self.iter;
        let &RRTAlgorithm {
            target,
            origin,
            ref obstacles,
            ..
        } = self;

        let root = CanvasBackend::with_canvas_object(canvas)
            .unwrap()
            .into_drawing_area();

        root.fill(&WHITE)?;

        for rect in obstacles {
            root.draw(&Rectangle::new(
                [
                    (
                        (rect.corner[0] * 10.0) as i32,
                        (rect.corner[1] * 10.0) as i32,
                    ),
                    (
                        ((rect.corner[0] + rect.size[0]) * 10.0) as i32,
                        ((rect.corner[1] + rect.size[1]) * 10.0) as i32,
                    ),
                ],
                ShapeStyle::from(&soft_red).filled(),
            ))?;
        }

        root.draw(&Circle::new(
            ((target[0] * 10.0) as i32, (target[1] * 10.0) as i32),
            10,
            ShapeStyle::from(&GREEN).filled(),
        ))?;

        root.draw(&Circle::new(
            ((origin[0] * 10.0) as i32, (origin[1] * 10.0) as i32),
            10,
            ShapeStyle::from(&MAGENTA).filled(),
        ))?;

        let mut vis = HashSet::new();

        for point in kd_tree.iter() {
            let mut curr_point = point;
            let mut series = Vec::new();

            while let Some(prev) = curr_point.connected().clone() {
                let node = kd_tree.get_point(prev);
                let p: &na::Point<f32, 2> = node.borrow();
                series.push(((p[0] * 10.0) as i32, (p[1] * 10.0) as i32));

                if vis.contains(&prev) {
                    break;
                }

                vis.insert(prev);
                curr_point = node;
            }

            let color = RGBColor(0, 0, 0).mix(0.5);

            if series.len() > 0 {
                root.draw(&PathElement::new(
                    series.clone(),
                    ShapeStyle::from(&color).filled(),
                ))?;
            }
        }

        let waypoints = if let Some(reached_idx) = reached_idx {
            let reached = kd_tree.get_point(reached_idx);

            let mut waypoints = Vec::new();
            let mut curr_point = reached;

            while let Some(prev) = curr_point.connected() {
                let prev = kd_tree.get_point(prev);
                waypoints.push(prev.point());
                curr_point = prev;
            }
            waypoints.push(from.point());
            waypoints.reverse();
            waypoints
        } else {
            return Ok(());
        };

        let series: Vec<_> = waypoints
            .into_iter()
            .map(|p| ((p.x * 10.0) as i32, (p.y * 10.0) as i32))
            .collect();

        root.draw(&PathElement::new(
            series.clone(),
            ShapeStyle::from(&BLUE).filled().stroke_width(5),
        ))?;

        Ok(())
    }
}
