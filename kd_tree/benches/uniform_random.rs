use criterion::{Criterion, BenchmarkId, criterion_group, criterion_main};
use nalgebra as na;

use kd_tree::KDTree;
use kd_tree::visitor::NearestVisitor;

fn gen_point() -> na::Point3<f32> {
    rand::random()
}

fn gen_points(n: usize) -> impl Iterator<Item = na::Point3<f32>> {
    std::iter::repeat_with(gen_point).take(n)
}

fn kd_tree_uniform_build(c: &mut Criterion) {
    let n = 100_000;
    let points: Vec<na::Point3<f32>> = gen_points(n).collect();

    c.bench_function(
        "kd_tree_uniform_build",
        |b| b.iter(|| {
            let tree = KDTree::new();
            tree.extend_vec(points.clone());
        }),
    );
}

fn kd_tree_uniform_query(c: &mut Criterion) {
    let n = 100_000;
    let tree = KDTree::new();
    tree.extend_vec(gen_points(n).collect());
    // assert_eq!(n, tree.size());

    c.bench_function(
        "kd_tree_uniform_query",
        |b| b.iter(|| {
            let vis = NearestVisitor::new();
            let query_p = gen_point();
            tree.query(&query_p, vis);
        }),
    );
}

fn kd_tree_uniform_insert(c: &mut Criterion) {
    let n = 100;

    c.bench_with_input(
        BenchmarkId::new("kd_tree_uniform_insert", n),
        &n,
        |b, &n| b.iter(|| {
            let tree = KDTree::new();
            for _ in 0..n {
                tree.insert(gen_point());
            }
        }),
    );
}

criterion_group!(benches, kd_tree_uniform_build, kd_tree_uniform_query, kd_tree_uniform_insert);
criterion_main!(benches);
