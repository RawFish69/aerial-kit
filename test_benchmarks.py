from aerial_kit.benchmarks import (
    benchmark_scenarios,
    markdown_table,
    run_controller_benchmarks,
)


def test_benchmark_harness_reports_a_controller_matrix() -> None:
    scenario = benchmark_scenarios()[1:2]  # step
    results = run_controller_benchmarks(("pid", "cascade", "l1_tecs"), scenario)
    assert {(r.controller, r.scenario) for r in results} == {
        ("pid", "step"),
        ("cascade", "step"),
        ("l1_tecs", "step"),
    }
    assert all(r.finite for r in results), [(r.controller, r.error) for r in results]
    table = markdown_table(results)
    assert "| Controller | Family | Scenario |" in table
    assert table.count("\n") == 5
