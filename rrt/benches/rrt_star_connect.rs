use criterion::{Criterion, BenchmarkId, criterion_group, criterion_main};

use nalgebra as na;

use rrt::{obstacle as obs, RRTBuilder, RRTStarConnectSolver};

fn point2(x: f32, y: f32) -> na::Point2<f32> {
    na::Point2::new(x, y)
}

fn vec2(x: f32, y: f32) -> na::Vector2<f32> {
    na::Vector2::from([x, y])
}

fn rrt_star_connect(c: &mut Criterion) {
    let target = point2(5.0, 5.0);
    let origin = point2(65.0, 65.0);

    let obstacles = &[
        obs::Rectangle::new(point2(10.0, 10.0), vec2(50.0, 50.0)),
        obs::Rectangle::new(point2(70.0, 10.0), vec2(50.0, 50.0)),
        obs::Rectangle::new(point2(10.0, 70.0), vec2(50.0, 50.0)),
        obs::Rectangle::new(point2(70.0, 70.0), vec2(50.0, 50.0)),
    ];

    let mut group = c.benchmark_group("RRT*_Connect");
    for max_iters in (4_000..=10_000).step_by(2_000) {
        group.bench_with_input(BenchmarkId::from_parameter(max_iters), &max_iters, |b, &max_iters| {
            b.iter(|| {
                RRTBuilder::<RRTStarConnectSolver, _, 2>::new(origin, target)
                    .extend_obstacles(obstacles)
                    .with_step_size(1.0)
                    .with_target_radius(1.0)
                    .with_max_stepping(1)
                    .with_max_iters(max_iters)
                    .with_update_radius(10.0)
                    .with_random_range_start(point2(0.0, 0.0))
                    .with_random_range_end(point2(130.0, 130.0))
                    .solve()
            });
        });
    }
    group.finish();
}

criterion_group!(benches, rrt_star_connect);
criterion_main!(benches);