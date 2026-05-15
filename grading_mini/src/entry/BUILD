load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "grading_main",
    srcs = ["grading_main.cc"],
    deps = [
        "//proto/grading:grading_run_config_cc_proto",
        "//proto/grading:metric_input_cc_proto",
        "//proto/grading:metric_output_cc_proto",
        "//proto/grading:sim_log_cc_proto",
        "//src/grading:grader",
        "//src/grading/metrics/example:planning_limit_checker",
        "//src/grading/metrics/example:speed_checker",
        "//src/grading/metrics/safety:regulatory_collision_checker",
        "//src/planning:simple_planner",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_protobuf//:protobuf",
        "@com_github_gabime_spdlog//:spdlog",
    ],
)
