load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "object_fusion_common",
    srcs = [
        "utils.cpp",
    ],
    hdrs = [
        "types.h",
        "utils.h",
    ],
    copts = ["-std=c++17"],
    features = [
        "treat_warnings_as_errors",
        "strict_clang_tidy_warnings",
        "strict_warnings",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/time",
        "//third_party/eigen",
    ],
)
