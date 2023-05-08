load("@//bazel/rules/licenses:licenses.bzl", "licenses")

licenses([
    "Mozilla Public License 2.0 (MPL-2.0)",
    "GNU General Public License v3.0 (GPL-3.0)",
    "GNU Lesser General Public License v2.1 (LGPL-2.1)",
])

alias(
    name = "eigen",
    actual = select(
        {
            "//conditions:default": "@eigen//:eigen",
        },
        no_match_error = "No known platform specified.",
    ),
    visibility = ["//visibility:public"],
)
