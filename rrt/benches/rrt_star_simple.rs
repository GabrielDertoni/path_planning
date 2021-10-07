use std::time::Duration;
use criterion::{Criterion, BenchmarkId, SamplingMode, criterion_group, criterion_main};

use nalgebra as na;

use rrt::{obstacle::Rectangle, RRTBuilder, RRTStarSolver, solvers::star::RRTStarIter};

fn point2(x: f32, y: f32) -> na::Point2<f32> {
    na::Point2::new(x, y)
}

fn vec2(x: f32, y: f32) -> na::Vector2<f32> {
    na::Vector2::from([x, y])
}

fn setup() -> (na::Point2<f32>, na::Point2<f32>, Vec<Rectangle<2>>) {
    let target = point2(5.0, 5.0);
    let origin = point2(65.0, 65.0);

    let obstacles = vec![
        Rectangle::new(point2(10.0, 10.0), vec2(50.0, 50.0)),
        Rectangle::new(point2(70.0, 10.0), vec2(50.0, 50.0)),
        Rectangle::new(point2(10.0, 70.0), vec2(50.0, 50.0)),
        Rectangle::new(point2(70.0, 70.0), vec2(50.0, 50.0)),
    ];

    (target, origin, obstacles)
}

fn rrt_star_simple(c: &mut Criterion) {
    let (target, origin, obstacles) = setup();

    let mut group = c.benchmark_group("RRT*");
    group.measurement_time(Duration::from_secs(5));
    group.sample_size(10);
    group.sampling_mode(SamplingMode::Flat);
    let max_iters = 60_000;
    
    group.bench_function(BenchmarkId::new("solve", max_iters), |b| {
        b.iter(|| {
            RRTBuilder::<RRTStarSolver, _, 2>::new(origin, target)
                .extend_obstacles(obstacles.iter())
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

fn rrt_star_rebalance(c: &mut Criterion) {
    let (target, origin, obstacles) = setup();

    let mut group = c.benchmark_group("RRT*_rebalance");
    group.measurement_time(Duration::from_secs(5));
    group.sample_size(10);
    group.sampling_mode(SamplingMode::Flat);
    let rebalance_factor = 2.0/3.0;

    let builder = RRTBuilder::new(origin, target)
        .extend_obstacles(obstacles.iter())
        .with_step_size(1.0)
        .with_target_radius(1.0)
        .with_max_stepping(1)
        .with_update_radius(1.0)
        .with_random_range_start(point2(0.0, 0.0))
        .with_random_range_end(point2(130.0, 130.0));

    for max_iters in (25_000..=100_000).step_by(25_000) {
        let builder = builder.clone().with_max_iters(max_iters);
        // How many nodes should we put in the Random Tree before rebalancing and benchmarking
        let to_rebalance = (max_iters as f32 * rebalance_factor) as usize;
        
        let mut iter = RRTStarIter::from_builder(builder.clone());
        for _ in iter.by_ref().zip(0..to_rebalance) {}
        
        group.bench_function(BenchmarkId::new("no_rebalance", max_iters), |b| {
            b.iter(|| {
                for _ in iter.clone() {}
            });
        });

        let mut iter = RRTStarIter::from_builder(builder.clone());
        for _ in iter.by_ref().zip(0..to_rebalance) {}
        iter.kd_tree.rebuild();

        group.bench_function(BenchmarkId::new("rebalance", max_iters), |b| {
            b.iter(|| {
                for _ in iter.clone() {}
            });
        });
    }
    group.finish();
}

fn rrt_star_rebalance_midway(c: &mut Criterion) {
    let (target, origin, obstacles) = setup();

    let mut group = c.benchmark_group("RRT*_rebalance_midway");
    group.measurement_time(Duration::from_secs(5));
    group.sample_size(10);
    group.sampling_mode(SamplingMode::Flat);
    let rebalance_factor = 2.0/3.0;

    let builder = RRTBuilder::new(origin, target)
        .extend_obstacles(obstacles.iter())
        .with_step_size(1.0)
        .with_target_radius(1.0)
        .with_max_stepping(1)
        .with_update_radius(1.0)
        .with_random_range_start(point2(0.0, 0.0))
        .with_random_range_end(point2(130.0, 130.0));

    for max_iters in (25_000..=100_000).step_by(25_000) {
        let builder = builder.clone().with_max_iters(max_iters);
        // How many nodes should we put in the Random Tree before rebalancing and benchmarking
        let to_rebalance = (max_iters as f32 * rebalance_factor) as usize;

        group.bench_function(BenchmarkId::new("solve", max_iters), |b| {
            b.iter(|| {
                builder.clone().solve();
            });
        });

        let iter = RRTStarIter::from_builder(builder.clone());

        group.bench_function(BenchmarkId::new("no_rebalance", max_iters), |b| {
            b.iter(|| {
                for _ in iter.clone() {}
            });
        });

        let iter = RRTStarIter::from_builder(builder);

        group.bench_function(BenchmarkId::new("rebalance", max_iters), |b| {
            b.iter(|| {
                let mut iter = iter.clone();
                for _ in iter.by_ref().zip(0..to_rebalance) {}
                iter.kd_tree.rebuild();
                for _ in iter.clone() {}
            });
        });   
    }
}

criterion_group!(benches, rrt_star_simple, rrt_star_rebalance, rrt_star_rebalance_midway);
criterion_main!(benches);