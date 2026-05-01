load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "grading_main",
    srcs = ["grading_main.cc"],
    deps = [
        "//proto/grading:metric_input_cc_proto",
        "//proto/grading:metric_output_cc_proto",
        "//src/grading:grader",
        "//src/grading/metrics/example:speed_checker",
        "@com_google_protobuf//:protobuf",
        "@com_github_gabime_spdlog//:spdlog",
    ],
)
