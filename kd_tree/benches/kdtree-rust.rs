use criterion::{Criterion, criterion_main, criterion_group};
use nalgebra as na;

use kd_tree::KDTree;
use kd_tree::visitor::{NearestVisitor, WithinRadiusVisitor};

fn gen_point() -> na::Point3<f32> {
    rand::random()
}

fn gen_points(n: usize) -> impl Iterator<Item = na::Point3<f32>> {
    std::iter::repeat_with(gen_point).take(n)
}

fn bench_creating_1000_node_tree(c: &mut Criterion) {
    c.bench_function("bench_creating_1000_node_tree", |b| {
        let len = 1000usize;
        let points: Vec<_> = gen_points(len).collect();

        b.iter(|| {
            let tree = KDTree::new();
            tree.extend_vec(points.clone());
        })
    });
}

fn bench_creating_100_000_node_tree(c: &mut Criterion) {
    c.bench_function("bench_creating_100_000_node_tree", |b| {
        let len = 100_000usize;
        let points: Vec<_> = gen_points(len).collect();

        b.iter(|| {
            let tree = KDTree::new();
            tree.extend_vec(points.clone());
        })
    });
}

fn bench_single_loop_times_for_1000_node_tree(c: &mut Criterion) {
    c.bench_function("bench_single_loop_times_for_1000_node_tree", |b| {
        let len = 1000usize;
        let points: Vec<_> = gen_points(len).collect();

        let query_p = points[0];

        let tree = KDTree::new();
        tree.extend_vec(points);

        b.iter(|| {
            let vis = NearestVisitor::new();
            tree.query(&query_p, vis);
        })
    });
}

fn bench_single_loop_times_for_100_000_node_tree(c: &mut Criterion) {
    c.bench_function("bench_single_loop_times_for_100_000_node_tree", |b| {
        let len = 1000_000usize;
        let points: Vec<_> = gen_points(len).collect();

        let query_p = points[0];

        let tree = KDTree::new();
        tree.extend_vec(points);

        b.iter(|| {
            let vis = NearestVisitor::new();
            tree.query(&query_p, vis);
        })
    });
}

fn bench_single_loop_times_for_1000_node_tree_within_1000(c: &mut Criterion) {
    c.bench_function(
        "bench_single_loop_times_for_1000_node_tree_within_1000",
        |b| {
            let len = 1000usize;
            let points: Vec<_> = gen_points(len).collect();

            let query_p = points[0];

            let tree = KDTree::new();
            tree.extend_vec(points);

            b.iter(|| {
                let vis = WithinRadiusVisitor::new(1000.0);
                tree.query(&query_p, vis);
            })
        },
    );
}

#[allow(dead_code)]
fn bench_creating_1000_000_node_tree(c: &mut Criterion) {
    c.bench_function("bench_creating_1000_000_node_tree", |b| {
        let len = 1000_000usize;
        let points: Vec<_> = gen_points(len).collect();

        b.iter(|| {
            let tree = KDTree::new();
            tree.extend_vec(points.clone());
        })
    });
}

fn bench_adding_same_node_to_1000_tree(c: &mut Criterion) {
    c.bench_function("bench_adding_same_node_to_1000_tree", |b| {
        let len = 1000usize;
        let points: Vec<_> = gen_points(len).collect();
        let tree = KDTree::new();
        tree.extend_vec(points);

        let point = gen_point();
        b.iter(|| {
            tree.insert(point);
        })
    });
}


fn bench_incrementally_building_the_1000_tree(c: &mut Criterion) {
    c.bench_function("bench_incrementally_building_the_1000_tree", |b| {
        b.iter(|| {
            let len = 1usize;
            let points: Vec<_> = gen_points(len).collect();
            let tree = KDTree::new();
            tree.extend_vec(points);
            for _ in 0..1000 {
                let point = gen_point();
                tree.insert(point);
            }
        })
    });
}

criterion_group!(
    kdtree_rust_comparison,
    bench_creating_1000_node_tree,
    bench_creating_100_000_node_tree,
    bench_single_loop_times_for_1000_node_tree,
    bench_single_loop_times_for_100_000_node_tree,
    bench_adding_same_node_to_1000_tree,
    bench_incrementally_building_the_1000_tree,
    bench_single_loop_times_for_1000_node_tree_within_1000
);
criterion_main!(kdtree_rust_comparison);

