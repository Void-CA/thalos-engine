//! Ratchet guard for file size.
//!
//! Production source is limited to 300 lines (the `#[cfg(test)]` tail is not
//! production code). Test files and `#[cfg(test)]` modules are limited to 400.
//! Existing offenders are recorded with their current baseline: they may
//! shrink, but they may not grow. Once an entry falls to/below the limit it
//! must be removed from the baseline, so the allowlist can only disappear.

use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

const PROD_MAX: usize = 300;
const TEST_MAX: usize = 400;

const PROD_BASELINE: &[(&str, usize)] = &[
    ("thalos-language-service/src/reactive_process.rs", 1423),
    ("thalos-language-service/src/registry.rs", 351),
    ("thalos-language-service/src/intelligence.rs", 975),
    ("thalos-lang/src/parser/mod.rs", 860),
    ("thalos-language-service/src/compiler.rs", 952),
    ("thalos-core/src/execution/session.rs", 704),
    ("thalos-language-service/src/reactive.rs", 623),
    ("thalos-language-service/src/checker.rs", 720),
    ("thalos-planning/src/motion/compiler.rs", 564),
    ("thalos-importer/src/urdf/elements.rs", 542),
    ("thalos-language-service/src/lib.rs", 445),
    ("thalos-language-service/src/evaluator.rs", 457),
    ("thalos-core/src/robot/executor.rs", 399),
    ("thalos-visual/src/mesh_loader.rs", 393),
    ("thalos-planning/src/program_edit.rs", 387),
    ("thalos-lang/src/ast/spanned.rs", 375),
    ("thalos-models/src/graph.rs", 334),
    ("thalos-core/src/analysis/region_grouper.rs", 333),
    ("thalos-language-service/src/offset.rs", 325),
    ("thalos-planning/src/resolver/motion_resolver.rs", 323),
    ("thalos-language-service/src/binder.rs", 415),
    ("thalos-visual/src/builder.rs", 316),
    ("thalos-planning/src/motion/move_l.rs", 315),
    ("thalos-math/src/geometry/unit_quaternion/model.rs", 315),
    ("thalos-core/src/robot/state.rs", 314),
    ("thalos-language-service/src/resolver.rs", 312),
    ("thalos-language-service/src/lowering/mod.rs", 307),
];

const TEST_BASELINE: &[(&str, usize)] = &[
    (
        "thalos-language-service/src/lowering/mod.rs [cfg(test)]",
        807,
    ),
    ("thalos-planning/src/motion/compiler.rs [cfg(test)]", 732),
    ("thalos-planning/src/program_edit.rs [cfg(test)]", 698),
    (
        "thalos-core/src/kinematics/jacobian/manipulability.rs [cfg(test)]",
        641,
    ),
    (
        "thalos-planning/src/resolver/motion_resolver.rs [cfg(test)]",
        629,
    ),
    ("thalos-core/src/models/scara/tests/jacobian.rs", 559),
    ("thalos-lang/tests/parser_tests.rs", 675),
    ("thalos-core/src/kinematics/inverse/tests/jt.rs", 526),
    ("thalos-core/src/models/planar_3r/tests/jacobian.rs", 484),
    ("thalos-core/src/models/scara/tests/geom_jacobian.rs", 483),
    (
        "thalos-language-service/src/reactive_eval.rs [cfg(test)]",
        479,
    ),
    ("thalos-core/src/execution/runtime.rs [cfg(test)]", 474),
    ("thalos-planning/src/motion/move_l.rs [cfg(test)]", 470),
    (
        "thalos-core/src/models/planar_3r/tests/geom_jacobian.rs",
        445,
    ),
    (
        "thalos-core/src/models/planar_2r/tests/geom_jacobian.rs",
        439,
    ),
    ("thalos-core/src/analysis/scoring.rs [cfg(test)]", 429),
    ("thalos-core/src/analysis/aggregator.rs [cfg(test)]", 419),
    ("thalos-core/src/execution/program.rs [cfg(test)]", 413),
];

#[derive(Default)]
struct Sizes {
    prod: BTreeMap<String, usize>,
    test: BTreeMap<String, usize>,
}

#[test]
fn production_and_test_sources_respect_the_ratchet() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .canonicalize()
        .expect("engine workspace root");
    let mut sizes = Sizes::default();
    collect(&root, &root, &mut sizes);

    check("production", &sizes.prod, PROD_BASELINE, PROD_MAX, &root);
    check("test", &sizes.test, TEST_BASELINE, TEST_MAX, &root);
}

fn collect(root: &Path, dir: &Path, sizes: &mut Sizes) {
    let entries = fs::read_dir(dir).unwrap_or_else(|e| panic!("read {}: {e}", dir.display()));
    for entry in entries {
        let entry = entry.expect("dir entry");
        let path = entry.path();
        let name = entry.file_name().to_string_lossy().to_string();
        if path.is_dir() {
            if name == "target" || name == ".git" {
                continue;
            }
            collect(root, &path, sizes);
            continue;
        }
        if path.extension().and_then(|e| e.to_str()) != Some("rs") {
            continue;
        }

        let relative = relative(root, &path);
        let content =
            fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
        let total = content.lines().count();

        if is_test_file(&path, &name) {
            if total > TEST_MAX {
                sizes.test.insert(relative, total);
            }
            continue;
        }

        let production_lines = content
            .lines()
            .position(|line| line.trim_start().starts_with("#[cfg(test)]"))
            .unwrap_or(total);
        if production_lines > PROD_MAX {
            sizes.prod.insert(relative.clone(), production_lines);
        }
        let test_lines = total.saturating_sub(production_lines);
        if test_lines > TEST_MAX {
            sizes
                .test
                .insert(format!("{relative} [cfg(test)]"), test_lines);
        }
    }
}

fn is_test_file(path: &Path, name: &str) -> bool {
    let in_tests = path
        .components()
        .any(|c| c.as_os_str() == "tests" || c.as_os_str() == "examples");
    in_tests || name == "tests.rs" || name.ends_with("_tests.rs")
}

fn relative(root: &Path, path: &Path) -> String {
    path.strip_prefix(root)
        .unwrap_or(path)
        .to_string_lossy()
        .replace('\\', "/")
}

fn check(
    label: &str,
    actual: &BTreeMap<String, usize>,
    baseline: &[(&str, usize)],
    limit: usize,
    root: &Path,
) {
    let baseline: BTreeMap<&str, usize> = baseline.iter().copied().collect();
    let mut failures = Vec::new();

    for (path, &size) in actual {
        match baseline.get(path.as_str()) {
            Some(&recorded) => {
                if size > recorded {
                    failures.push(format!(
                        "{label} file grew: {path} is {size} lines, baseline {recorded}"
                    ));
                }
            }
            None => failures.push(format!(
                "{label} file exceeds {limit} lines and is not in the baseline: {path} ({size})"
            )),
        }
    }

    for path in baseline.keys() {
        if !actual.contains_key(*path) {
            let source = path.strip_suffix(" [cfg(test)]").unwrap_or(path);
            if !root.join(source).is_file() {
                failures.push(format!("stale baseline path does not exist: {path}"));
            } else {
                failures.push(format!(
                    "stale baseline entry is now within the limit; remove it: {path}"
                ));
            }
        }
    }

    assert!(
        failures.is_empty(),
        "File-size ratchet failed:\n{}",
        failures.join("\n")
    );
}
