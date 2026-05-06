#!/usr/bin/env bash
set -euo pipefail

project_name=""
profile=""

usage() {
  echo "Usage: $0 --name <project-name> --profile <python|cpp|document>"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --name)
      project_name="$2"
      shift 2
      ;;
    --profile)
      profile="$2"
      shift 2
      ;;
    *)
      usage
      exit 1
      ;;
  esac
done

if [[ -z "$project_name" || -z "$profile" ]]; then
  usage
  exit 1
fi

if [[ ! -d "profiles/${profile}" ]]; then
  echo "Unknown profile: $profile"
  exit 2
fi

requirements_manifest="config/requirements.list"

sed -i "s/^name: .*/name: \"${project_name}\"/" config/project.yaml
sed -i "s/^profile: .*/profile: \"${profile}\" # python | cpp | document/" config/project.yaml
sed -i "s/^- Name: .*/- Name: ${project_name}/" STATE.md
sed -i "s/^- Profile: .*/- Profile: ${profile}/" STATE.md

if ! grep -q '^  requirements:' config/project.yaml; then
  printf '\n  requirements: "%s"\n' "$requirements_manifest" >> config/project.yaml
fi

if ! grep -q '^## Requirements' STATE.md; then
  cat <<'EOF' >> STATE.md

## Requirements

- Manifest: config/requirements.list
- Check command: scripts/helpers/check-requirements.sh
- Installation status: _Not checked yet._
EOF
fi

mkdir -p .agents
cp "profiles/${profile}/README.profile.md" "docs/templates/active-profile.md"

echo "Project bootstrapped."
echo "Name: $project_name"
echo "Profile: $profile"
echo "Requirements manifest: $requirements_manifest"
