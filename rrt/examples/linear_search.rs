use std::ops::Range;

use plotters::prelude::*;
use nalgebra as na;

use rrt::{obstacle as obs, rrt_solve_points_vec};

fn point2(x: f32, y: f32) -> na::Point2<f32> {
    na::Point2::new(x, y)
}

fn vec2(x: f32, y: f32) -> na::Vector2<f32> {
    na::Vector2::from([x, y])
}

fn main() -> Result<(), Box<dyn std::error::Error>> {

    let obstacles = &[
        obs::Rectangle::new(point2(10.0, 10.0), vec2(50.0, 50.0)),
        obs::Rectangle::new(point2(70.0, 10.0), vec2(50.0, 50.0)),
        obs::Rectangle::new(point2(10.0, 70.0), vec2(50.0, 50.0)),
        obs::Rectangle::new(point2(70.0, 70.0), vec2(50.0, 50.0)),
    ];

    let root = BitMapBackend::new("test.png", (1300, 1300)).into_drawing_area();

    root.fill(&WHITE)?;

    for rect in obstacles {
        root.draw(&Rectangle::new(
            [
                ((rect.corner[0] * 10.0) as i32, (rect.corner[1] * 10.0) as i32),
                (((rect.corner[0] + rect.size[0]) * 10.0) as i32, ((rect.corner[1] + rect.size[1]) * 10.0) as i32)
            ],
            ShapeStyle::from(&RED).filled(),
        ))?;
    }

    let target = na::Point2::new(5.0, 5.0);
    let origin = na::Point2::new(65.0, 65.0);

    let res = rrt_solve_points_vec(
        origin,
        target,
        obstacles,
        Range { start: na::Point2::new(0.0, 0.0), end: na::Point2::new(130.0, 130.0) },
        1.0,
        5,
        0.5,
    );

    assert_eq!(res.n_points, res.points.len());

    eprintln!("Used {} points", res.n_points);
    let path = res.result.unwrap();

    let series: Vec<_> = path.waypoints
        .into_iter()
        .map(|p| ((p.x * 10.0) as i32, (p.y * 10.0) as i32))
        .collect();

    root.draw(&PathElement::new( 
        series.clone(),
        ShapeStyle::from(&BLUE).filled(),
    ))?;

    for coord in series {
        root.draw(&Circle::new(
            coord,
            3,
            ShapeStyle::from(&BLUE).filled(),
        ))?;
    }

    Ok(())
}