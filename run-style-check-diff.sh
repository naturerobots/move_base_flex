#!/bin/bash

set +e

# We need to add a new remote for the upstream master, since this script could
# be running in a personal fork of the repository which has out of date branches.
git remote add upstream https://github.com/magazino/move_base_flex
git fetch upstream

# Work out the newest common ancestor between the detached HEAD that this CI job
# has checked out, and the upstream target branch (which will typically be
# `upstream/master` or one of the ROS version branches).
# `${CI_MERGE_REQUEST_TARGET_BRANCH_NAME}` is only defined if we’re running in
# a merge request pipeline; fall back to `${CI_DEFAULT_BRANCH}` otherwise.
newest_common_ancestor_sha=$(diff --old-line-format='' --new-line-format='' <(git rev-list --first-parent upstream/${CI_MERGE_REQUEST_TARGET_BRANCH_NAME:-${CI_DEFAULT_BRANCH}}) <(git rev-list --first-parent HEAD) | head -1)
git diff -U0 --no-color "${newest_common_ancestor_sha}" | ./clang-format-diff.py -format=.clang-format -binary "clang-format" -p1

# The style check is not infallible. The clang-format configuration cannot
# perfectly describe GLib’s coding style: in particular, it cannot align
# function arguments. The documented coding style for GLib takes priority over
# clang-format suggestions. Hopefully we can eventually improve clang-format to
# be configurable enough for our coding style. That’s why this CI check is OK
# to fail: the idea is that people can look through the output and ignore it if
# it’s wrong. (That situation can also happen if someone touches pre-existing
# badly formatted code and it doesn’t make sense to tidy up the wider coding
# style with the changes they’re making.)
echo ""
echo "Note that clang-format output is advisory and cannot always match the GLib coding style, documented at"
echo "   https://gitlab.gnome.org/GNOME/gtk/blob/master/docs/CODING-STYLE"
echo "Warnings from this tool can be ignored in favour of the documented coding style,"
echo "or in favour of matching the style of existing surrounding code."
