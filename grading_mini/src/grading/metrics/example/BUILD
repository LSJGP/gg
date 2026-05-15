load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "planning_limit_checker",
    srcs = ["planning_limit_checker.cc"],
    hdrs = ["planning_limit_checker.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//proto/grading/metrics:example_metric_cc_proto",
        "//src/grading:metric_base",
        "//src/grading:metric_register",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_github_gabime_spdlog//:spdlog",
    ],
    alwayslink = True,
)

cc_library(
    name = "speed_checker",
    srcs = ["speed_checker.cc"],
    hdrs = ["speed_checker.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//proto/grading/metrics:example_metric_cc_proto",
        "//src/grading:metric_base",
        "//src/grading:metric_register",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_github_gabime_spdlog//:spdlog",
    ],
    alwayslink = True,
)
