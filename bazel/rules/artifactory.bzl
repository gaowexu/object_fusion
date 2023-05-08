"""Rules and macros to facilitate usage of binary artifacts hosted on artifactory"""

load("@//bazel/rules:http_convert_to_linux_archive.bzl", "http_convert_to_linux_archive")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file", "http_jar")

def _get_url(repo, path):
    """Join repo and path with / to obtain the url"""
    return repo.rstrip("/") + "/" + path

def artifactory_archive(name, repo, path, **kwargs):
    http_archive(
        name = name,
        url = _get_url(repo, path),
        **kwargs
    )

def artifactory_file(name, repo, path, **kwargs):
    http_file(
        name = name,
        urls = [_get_url(repo, path)],
        **kwargs
    )

def artifactory_convert_to_linux_archive(name, repo, path, **kwargs):
    http_convert_to_linux_archive(
        name = name,
        url = _get_url(repo, path),
        **kwargs
    )

def artifactory_jar(name, repo, path, **kwargs):
    http_jar(
        name = name,
        urls = [_get_url(repo, path)],
        **kwargs
    )
