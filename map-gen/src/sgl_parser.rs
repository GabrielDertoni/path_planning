use std::collections::HashMap;

use nalgebra as na;

use rrt::obstacle::Polytope;

pub fn parse_sgl<const N: usize>(s: &str) -> Option<HashMap<usize, Polytope<N>>> {
    let mut lines = s.lines();
    let header = lines.next()?;

    if header != "<number of polygons>" {
        return None;
    }

    let n_polygons = lines.next()?.parse().ok()?;
    let mut polygons = HashMap::with_capacity(n_polygons);

    let coord_names = ["x", "y", "z"];

    for _ in 0..n_polygons {
        let poly_header = lines.next()?;
        let inner_attributes = poly_header
            .trim_matches(|c| c == '<' || c == '>')
            .split(',');

        let is_match = inner_attributes
            .clone()
            .zip(&coord_names[..N])
            .all(|(actual, &expected)| actual.trim_matches(|c| c == ' ' || c == '.') == expected);

        if !is_match {
            return None;
        }

        let map: HashMap<&str, usize> = inner_attributes
            .skip(N)
            .map(|s| -> Option<(&str, usize)> {
                let mut it = s.split('=').map(str::trim);
                Some((it.next()?, it.next()?.parse().ok()?))
            })
            .collect::<Option<_>>()?;

        let n = *map.get("n")?;
        let id = *map.get("id")?;

        let mut coords: Vec<Vec<f32>> = Vec::with_capacity(N);
        for _ in 0..N {
            coords.push(
                lines
                    .next()?
                    .split(',')
                    .map(|coord| coord.parse().ok())
                    .collect::<Option<_>>()?,
            );

            if coords[coords.len() - 1].len() != n {
                return None;
            }
        }

        let mut vertices = Vec::new();
        for i in 0..n {
            vertices.push(na::Point::from(na::SVector::from_iterator(
                coords.iter().map(|coord| coord[i]),
            )));
        }
        polygons.insert(id, Polytope { vertices });
    }

    Some(polygons)
}
