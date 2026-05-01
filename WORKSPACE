workspace(name = "grading_mini")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# ---- Rules CC ----
http_archive(
    name = "rules_cc",
    strip_prefix = "rules_cc-0.0.9",
    urls = ["https://github.com/bazelbuild/rules_cc/releases/download/0.0.9/rules_cc-0.0.9.tar.gz"],
)

# ---- Protobuf (v25 - stable WORKSPACE support) ----
http_archive(
    name = "com_google_protobuf",
    strip_prefix = "protobuf-25.3",
    urls = ["https://github.com/protocolbuffers/protobuf/archive/refs/tags/v25.3.tar.gz"],
)
load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")
protobuf_deps()

# ---- Abseil ----
http_archive(
    name = "com_google_absl",
    strip_prefix = "abseil-cpp-20240116.2",
    urls = ["https://github.com/abseil/abseil-cpp/archive/refs/tags/20240116.2.tar.gz"],
)

# ---- spdlog ----
http_archive(
    name = "com_github_gabime_spdlog",
    build_file = "//third_party:spdlog.BUILD",
    strip_prefix = "spdlog-1.13.0",
    urls = ["https://github.com/gabime/spdlog/archive/refs/tags/v1.13.0.tar.gz"],
)

# ---- fmtlib (spdlog dependency) ----
http_archive(
    name = "com_github_fmtlib_fmt",
    build_file = "//third_party:fmt.BUILD",
    strip_prefix = "fmt-10.2.1",
    urls = ["https://github.com/fmtlib/fmt/archive/refs/tags/10.2.1.tar.gz"],
)
