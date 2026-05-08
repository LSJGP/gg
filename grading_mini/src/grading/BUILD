load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "macros",
    hdrs = ["macros.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "metric_result_payload",
    hdrs = ["metric_result_payload.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "metric_base",
    srcs = ["metric_base.cc"],
    hdrs = ["metric_base.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":metric_result_payload",
        "//proto/grading:metric_input_cc_proto",
        "//proto/grading:metric_output_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_protobuf//:protobuf",
        "@com_github_gabime_spdlog//:spdlog",
    ],
)

cc_library(
    name = "metric_factory",
    srcs = ["metric_factory.cc"],
    hdrs = ["metric_factory.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":macros",
        ":metric_base",
        "@com_google_absl//absl/status:statusor",
        "@com_github_gabime_spdlog//:spdlog",
    ],
)

cc_library(
    name = "metric_register",
    hdrs = ["metric_register.h"],
    visibility = ["//visibility:public"],
    deps = [":metric_factory"],
)

cc_library(
    name = "metric_scheduling_policy",
    srcs = ["metric_scheduling_policy.cc"],
    hdrs = ["metric_scheduling_policy.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "metric_manager",
    srcs = ["metric_manager.cc"],
    hdrs = ["metric_manager.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":macros",
        ":metric_base",
        ":metric_result_payload",
        ":metric_scheduling_policy",
        "//proto/grading:metric_input_cc_proto",
        "//proto/grading:metric_output_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_github_gabime_spdlog//:spdlog",
    ],
)

cc_library(
    name = "grader",
    srcs = ["grader.cc"],
    hdrs = ["grader.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":macros",
        ":metric_factory",
        ":metric_manager",
        "//proto/grading:metric_input_cc_proto",
        "//proto/grading:metric_output_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_github_gabime_spdlog//:spdlog",
    ],
)
