load("@//bazel/rules/licenses:licenses.bzl", "licenses")

licenses(["MIT License (MIT)"])

alias(
    name = "gtest",
    actual = select(
        {
            "//conditions:default": "@google_test//:gtest",
        },
        no_match_error = "No known platform specified.",
    ),
    visibility = ["//visibility:public"],
)

alias(
    name = "gtest_main",
    actual = select(
        {
            "//conditions:default": "@google_test//:gtest_main",
        },
        no_match_error = "No known platform specified.",
    ),
    visibility = ["//visibility:public"],
)
