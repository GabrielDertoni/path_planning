use std::collections::HashSet;
use std::borrow::Borrow;
use std::env;

use plotters::prelude::*;
use nalgebra as na;

use rrt::{obstacle as obs, RRTBuilder, RRTStarSolver};

fn point2(x: f32, y: f32) -> na::Point2<f32> {
    na::Point2::new(x, y)
}

fn vec2(x: f32, y: f32) -> na::Vector2<f32> {
    na::Vector2::from([x, y])
}

fn main() -> Result<(), Box<dyn std::error::Error>> {

    let max_iters = env::args()
        .position(|arg| arg == "--max-iters" || arg == "-i")
        .and_then(|pos| env::args().nth(pos + 1)?.parse().ok())
        .unwrap_or(5_000);

    let plot = env::args()
        .any(|arg| arg == "--plot" || arg == "-p");

    let soft_red = RGBColor(200, 50, 50);

    let obstacles = &[
        obs::Rectangle::new(point2(10.0, 10.0), vec2(50.0, 50.0)),
        obs::Rectangle::new(point2(70.0, 10.0), vec2(50.0, 50.0)),
        obs::Rectangle::new(point2(10.0, 70.0), vec2(50.0, 50.0)),
        obs::Rectangle::new(point2(70.0, 70.0), vec2(50.0, 50.0)),
    ];

    let target = point2(5.0, 5.0);
    let origin = point2(65.0, 65.0);

    let res = RRTBuilder::<RRTStarSolver, _, 2>::new(origin, target)
        .extend_obstacles(obstacles)
        .with_step_size(1.0)
        .with_target_radius(1.0)
        .with_max_stepping(1)
        .with_max_iters(max_iters)
        .with_update_radius(1.0)
        .with_random_range_start(point2(0.0, 0.0))
        .with_random_range_end(point2(130.0, 130.0))
        .solve();

    eprintln!("Used {} points", res.n_points);
    if let Some(rrt::Path { cost: Some(cost), .. }) = &res.result {
        println!("Cost: {}", cost);
    }

    if plot {

        let mut avg = 0.0;
        let mut max = 0;
        for node in &res.graph.points {
            let n_children = node.children().len();
            avg += n_children as f32;
            if n_children > max {
                max = n_children;
            }
        }
        println!("Average number of children is {}", avg / res.graph.points.len() as f32);
        println!("Max number of children is {}", max);

        let root = BitMapBackend::new("test.png", (1300, 1300)).into_drawing_area();

        root.fill(&WHITE)?;

        for rect in obstacles {
            root.draw(&Rectangle::new(
                [
                    ((rect.corner[0] * 10.0) as i32, (rect.corner[1] * 10.0) as i32),
                    (((rect.corner[0] + rect.size[0]) * 10.0) as i32, ((rect.corner[1] + rect.size[1]) * 10.0) as i32)
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

        for point in &res.graph.points {
            let mut curr_point = point;
            let mut series = Vec::new();

            while let Some(prev) = curr_point.connected().clone() {
                let node = &res.graph.points[prev];
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

        if res.result.is_none() {
            eprintln!("No solution was found!");
            return Ok(());
        }

        let path = res.result.unwrap();

        let series: Vec<_> = path.waypoints
            .into_iter()
            .map(|p| ((p.x * 10.0) as i32, (p.y * 10.0) as i32))
            .collect();

        root.draw(&PathElement::new( 
            series.clone(),
            ShapeStyle::from(&BLUE).filled().stroke_width(3),
        ))?;
    }

    Ok(())
}