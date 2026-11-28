"""Stage GitHub ROS builds in the editable workspace before registering them."""

from dataclasses import dataclass
from pathlib import Path
import json
import os
import re
import shlex
import shutil
import subprocess
import tempfile
from urllib.parse import quote, unquote, urlsplit


@dataclass(frozen=True)
class GitHubLink:
    url: str
    name: str
    kind: str = ""
    ref_path: tuple = ()

    @property
    def page(self):
        if not self.kind:
            return self.url
        return "/".join((self.url.removesuffix(".git"), self.kind, *(quote(part, safe="") for part in self.ref_path)))


def github_link(value):
    value = value.strip()
    if value.startswith("github.com/"):
        value = "https://" + value
    ssh = re.fullmatch(r"git@github\.com:([A-Za-z0-9_.-]+)/([A-Za-z0-9_.-]+)", value)
    if ssh:
        owner, repository = ssh.groups()
        kind, extra = "", []
    else:
        link = urlsplit(value)
        if link.scheme not in ("https", "http") or link.netloc.lower() not in ("github.com", "www.github.com"):
            raise ValueError("Paste a github.com repository, branch, tag, commit, or pull request link.")
        parts = unquote(link.path).strip("/").split("/")
        if len(parts) < 2:
            raise ValueError("The GitHub link needs an owner and repository.")
        owner, repository = parts[:2]
        kind, *extra = parts[2:] or [""]
        if kind == "releases" and extra[:1] == ["tag"]:
            kind, extra = "tree", extra[1:]
        elif kind not in ("tree", "blob", "commit", "pull"):
            kind, extra = "", []
    for part in (owner, repository):
        if part in ("", ".", "..") or not re.fullmatch(r"[A-Za-z0-9_.-]+", part):
            raise ValueError("Invalid GitHub owner or repository.")
    repository = repository.removesuffix(".git")
    if not repository or repository in (".", ".."):
        raise ValueError("Invalid GitHub repository name.")
    if kind not in ("", "tree", "blob", "commit", "pull") or (kind and not extra):
        raise ValueError("Use a repository link or a branch, tag, commit, or pull request link.")
    if any(part in ("", ".", "..") or any(ord(char) < 32 for char in part) for part in extra):
        raise ValueError("Invalid path in the GitHub link.")
    if kind == "pull" and (not extra[0].isdigit() or extra[0].startswith("0")):
        raise ValueError("Invalid GitHub pull request number.")
    if kind == "commit" and not re.fullmatch(r"[0-9a-fA-F]{7,40}", extra[0]):
        raise ValueError("Invalid GitHub commit hash.")
    url = f"git@github.com:{owner}/{repository}.git" if ssh else f"https://github.com/{owner}/{repository}.git"
    return GitHubLink(url, repository, kind, tuple(extra))


def run_command(command, log, *, env=None, timeout=None, capture=True):
    result = subprocess.run(command, stdout=subprocess.PIPE if capture else log, stderr=log, text=True,
                            env=env, timeout=timeout, check=False)
    if result.stdout:
        log.write(result.stdout)
        log.flush()
    if result.returncode:
        raise RuntimeError(f"{Path(command[0]).name} exited with code {result.returncode}.")
    return (result.stdout or "").strip()


def child_path(root, path):
    candidate = (root / path).resolve()
    if not candidate.is_relative_to(root.resolve()):
        raise ValueError("The selected source folder must be inside the checkout.")
    return candidate


def checkout(link, source, log, previous):
    git = shutil.which("git")
    if not git:
        raise RuntimeError("Git is required to import a GitHub module.")
    env = dict(os.environ, GIT_TERMINAL_PROMPT="0")
    config = []
    if link.url.startswith("https://"):
        for cli in ("gh", "gh.exe"):
            gh = shutil.which(cli)
            if gh and subprocess.run([gh, "auth", "status", "--hostname", "github.com"],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                     timeout=15, check=False).returncode == 0:
                config = ["-c", f"credential.https://github.com.helper=!{shlex.quote(gh)} auth git-credential"]
                break

    def command(*args):
        return run_command([git, *config, "-C", str(source), *args], log, env=env, timeout=300)

    if previous and Path(previous["source"]).is_dir():
        status = run_command([git, "-C", previous["source"], "status", "--porcelain"], log, timeout=30)
        if status:
            raise ValueError("The current checkout has local edits. Commit or stash them before updating from GitHub.")
    source.mkdir()
    command("init", "--quiet")
    command("remote", "add", "origin", link.url)
    ref, subpath = "HEAD", ()
    if link.kind in ("tree", "blob"):
        refs = {line.split("\t", 1)[1] for line in command("ls-remote", "--heads", "--tags", "origin").splitlines()
                if "\t" in line}
        for length in range(len(link.ref_path), 0, -1):
            candidate = "/".join(link.ref_path[:length])
            matches = [f"refs/{prefix}/{candidate}" for prefix in ("heads", "tags")]
            ref = next((item for item in matches if item in refs), "")
            if ref:
                subpath = link.ref_path[length:]
                break
        if not ref:
            ref = link.ref_path[0]
            subpath = link.ref_path[1:]
        if link.kind == "blob":
            subpath = subpath[:-1]
    elif link.kind == "commit":
        ref = link.ref_path[0]
    elif link.kind == "pull":
        ref = f"refs/pull/{link.ref_path[0]}/head"
    try:
        command("fetch", "--no-tags", "origin", "--", ref)
        command("checkout", "--quiet", "--detach", "FETCH_HEAD")
        command("submodule", "update", "--init", "--recursive")
    except RuntimeError as error:
        raise RuntimeError(f"{error} Check the link and your Git credentials. Private repositories can use gh auth login.") from error
    commit = command("rev-parse", "HEAD")
    changes = ""
    if previous and Path(previous["source"]).is_dir() and previous.get("commit"):
        command("fetch", "--no-tags", "--", previous["source"], previous["commit"])
        changes = command("diff", "--stat", "FETCH_HEAD", "HEAD")
        command("diff", "FETCH_HEAD", "HEAD")
    selected = child_path(source, Path(*subpath))
    if not selected.is_dir():
        raise ValueError("The GitHub link does not point to a source folder.")
    return selected, commit, changes


def workspace_build(root, workspace, source, output, name, log):
    if not shutil.which("colcon"):
        raise RuntimeError("Install the ROS build tools with sudo apt install python3-colcon-common-extensions build-essential.")
    install = Path(tempfile.mkdtemp(prefix="build-", dir=output)) / "install"
    launcher = root / "bin/asurt"
    prefix = ["bash", str(launcher), "exec"] if launcher.is_file() else []
    env = dict(os.environ, ASURT_ROOT=str(root), ASURT_WORKSPACE=str(workspace))
    listing = run_command([*prefix, "colcon", "list", "--base-paths", str(source)], log, env=env)
    packages = {}
    for line in listing.splitlines():
        fields = line.split("\t")
        if len(fields) != 3:
            continue
        package, path, kind = fields
        if kind.startswith("(ros.") and re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", package):
            candidate = Path(path).resolve()
            if candidate.is_relative_to(source.resolve()):
                packages[package] = candidate
    if not packages:
        raise ValueError(f"{name} does not contain a ROS package that colcon can build.")
    run_command([*prefix, "colcon", "--log-base", str(install.parent / "log"), "build",
                 "--base-paths", str(source), "--packages-select", *packages,
                 "--build-base", str(install.parent / "build"), "--install-base", str(install),
                 "--event-handlers", "console_direct+"], log, env=env, capture=False)
    executables = {}
    for package, directory in packages.items():
        for binary in sorted((install / package / "lib" / package).glob("*")):
            if binary.is_file() and os.access(binary, os.X_OK):
                executables[f"{package}/{binary.name}"] = (binary, directory)
    if not executables or not (install / "local_setup.bash").is_file():
        raise ValueError("The build did not install a ROS executable under lib/<package>.")
    return install / "local_setup.bash", executables


def import_module(root, workspace, value, log_path, previous=None, *, rebuild=False):
    link = github_link(value)
    parent = workspace / "Modules" / "GitHub" / link.name
    parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log:
        if rebuild:
            source = Path(previous["source"])
            selected = child_path(source, previous.get("subpath", "."))
            commit, changes = previous["commit"], ""
        else:
            source = Path(tempfile.mkdtemp(prefix="checkout-", dir=parent)) / "source"
            selected, commit, changes = checkout(link, source, log, previous)
        setup, executables = workspace_build(root, workspace, selected, source.parent, link.name, log)
    return {
        "setup": setup, "executables": executables, "changes": changes,
        "github": {"link": link.page, "source": str(source), "commit": commit,
                   "subpath": str(selected.relative_to(source.resolve()))},
    }


def inferred_fields(package, directory):
    label = package.replace("_", " ").replace("-", " ").title()
    groups = {"control": "Control", "controller": "Control", "planning": "PathPlanning", "planner": "PathPlanning",
              "perception": "Perception", "slam": "SLAM", "localization": "SLAM"}
    group = next((groups[token] for token in re.split(r"[_-]", package.lower()) if token in groups), "Custom")
    fields = {"id": package, "label": label, "group": group, "drive_topic": "/drive" if group == "Control" else ""}
    config = directory / "asurt-module.json"
    if config.is_file():
        values = json.loads(config.read_text(encoding="utf-8"))
        for key in (*fields, "topic", "dependencies", "arguments"):
            value = values.get(key) if isinstance(values, dict) else None
            if isinstance(value, str):
                fields[key] = value
    return fields
