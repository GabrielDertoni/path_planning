use std::ops::Range;

use plotters::prelude::*;
use nalgebra as na;

use rrt::{ rrt_solve, obstacle::{ Obstacle, Sphere } };

fn main() -> Result<(), Box<dyn std::error::Error>> {

    let root = BitMapBackend::new("test.png", (500, 200)).into_drawing_area();
    root.fill(&WHITE)?;
    // Draw an circle on the drawing area
    root.draw(&Circle::new(
        (200, 100),
        50,
        Into::<ShapeStyle>::into(&RED).filled(),
    ))?;
    root.draw(&Circle::new(
        (300, 100),
        50,
        Into::<ShapeStyle>::into(&RED).filled(),
    ))?;

    root.draw(&Circle::new(
        (100, 100),
        5,
        Into::<ShapeStyle>::into(&CYAN).filled(),
    ))?;

    root.draw(&Circle::new(
        (400, 100),
        5,
        Into::<ShapeStyle>::into(&MAGENTA).filled(),
    ))?;

    let obstacle1 = Sphere::new(na::Point2::new(1.0, 0.0), 0.5);
    let obstacle2 = Sphere::new(na::Point2::new(2.0, 0.0), 0.5);
    let target = na::Point2::new(3.0, 0.0);
    let origin = na::Point2::new(0.0, 0.0);

    let path = rrt_solve::<Box<dyn Obstacle<2>>, 2>(
        origin,
        target,
        &[Box::new(obstacle1), Box::new(obstacle2)],
        Range { start: na::Point2::new(-1.0, -1.0), end: na::Point2::new(4.0, 1.0) },
        0.1,
        3,
        0.1,
        10,
    );

    /*
    let mut vis = HashSet::new();

    for point in points {
        let mut curr_point = point;
        let mut series = Vec::new();

        while let Some(prev) = curr_point.connected.clone() {
            let p = prev.p;
            series.push(((p.x * 100.0) as i32 + 100, (p.y * 100.0) as i32 + 100));

            if vis.contains(&(Rc::as_ptr(&prev) as usize)) {
                break;
            }

            vis.insert(Rc::as_ptr(&prev) as usize);
            curr_point = prev;
        }

        if series.len() > 0 {
            root.draw(&PathElement::new( 
                series.clone(),
                Into::<ShapeStyle>::into(&BLUE).filled(),
            ))?;

            for coord in series {
                root.draw(&Circle::new(
                    coord,
                    3,
                    Into::<ShapeStyle>::into(&BLUE).filled(),
                ))?;
            }
        }
    }
    */

    let series: Vec<_> = path.waypoints
        .into_iter()
        .map(|p| ((p.x * 100.0) as i32 + 100, (p.y * 100.0) as i32 + 100))
        .collect();

    root.draw(&PathElement::new( 
        series.clone(),
        Into::<ShapeStyle>::into(&BLUE).filled(),
    ))?;


    for coord in series {
        root.draw(&Circle::new(
            coord,
            3,
            Into::<ShapeStyle>::into(&BLUE).filled(),
        ))?;
    }
     

    Ok(())
}