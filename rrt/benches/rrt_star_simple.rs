use std::time::Duration;
use criterion::{Criterion, BenchmarkId, SamplingMode, criterion_group, criterion_main};

use nalgebra as na;

use rrt::{obstacle as obs, RRTBuilder, RRTStarSolver};

fn point2(x: f32, y: f32) -> na::Point2<f32> {
    na::Point2::new(x, y)
}

fn vec2(x: f32, y: f32) -> na::Vector2<f32> {
    na::Vector2::from([x, y])
}

fn rrt_star_full(c: &mut Criterion) {
    let target = point2(5.0, 5.0);
    let origin = point2(65.0, 65.0);

    let obstacles = &[
        obs::Rectangle::new(point2(10.0, 10.0), vec2(50.0, 50.0)),
        obs::Rectangle::new(point2(70.0, 10.0), vec2(50.0, 50.0)),
        obs::Rectangle::new(point2(10.0, 70.0), vec2(50.0, 50.0)),
        obs::Rectangle::new(point2(70.0, 70.0), vec2(50.0, 50.0)),
    ];

    let mut group = c.benchmark_group("RRT*");
    group.measurement_time(Duration::from_secs(10));
    group.sample_size(10);
    group.sampling_mode(SamplingMode::Flat);
    let max_iters = 30_000;
    
    group.bench_function(BenchmarkId::from_parameter(max_iters), |b| {
        b.iter(|| {
            RRTBuilder::<RRTStarSolver, _, 2>::new(origin, target)
                .extend_obstacles(obstacles)
                .with_step_size(1.0)
                .with_target_radius(1.0)
                .with_max_stepping(1)
                .with_max_iters(max_iters)
                .with_update_radius(1.0)
                .with_random_range_start(point2(0.0, 0.0))
                .with_random_range_end(point2(130.0, 130.0))
                .solve()
        });
    });
    
    group.finish();
}

criterion_group!(benches, rrt_star_full);
criterion_main!(benches);