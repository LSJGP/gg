load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "regulatory_collision_checker",
    srcs = ["regulatory_collision_checker.cc"],
    hdrs = ["regulatory_collision_checker.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//src/grading:metric_base",
        "//src/grading:metric_register",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_github_gabime_spdlog//:spdlog",
    ],
    alwayslink = True,
)
