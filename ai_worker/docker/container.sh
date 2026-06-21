#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P )"
AI_WORKER_SOURCE_DIR="${AI_WORKER_SOURCE_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd -P)}"
AI_WORKER_WORKSPACE_DIR="${AI_WORKER_WORKSPACE_DIR:-${SCRIPT_DIR}/workspace}"
export AI_WORKER_SOURCE_DIR
export AI_WORKER_WORKSPACE_DIR
CONTAINER_NAME="ai_worker"
GITHUB_RELEASES_API="https://api.github.com/repos/ROBOTIS-GIT/ai_worker/releases/latest"
META_PACKAGE_XML="${SCRIPT_DIR}/../ffw/package.xml"

# Function to display help
show_help() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  help                    Show this help message"
    echo "  start                   Start the container"
    echo "  enter                   Enter the running container"
    echo "  stop                    Stop the container"
    echo ""
    echo "Examples:"
    echo "  $0 start                Start container"
    echo "  $0 enter                Enter the running container"
    echo "  $0 stop                 Stop the container"
}

ensure_source_tree() {
    if [ ! -f "${AI_WORKER_SOURCE_DIR}/ffw_bringup/package.xml" ] ||
       [ ! -f "${AI_WORKER_SOURCE_DIR}/ffw_teleop/package.xml" ]; then
        echo "Error: AI_WORKER_SOURCE_DIR does not look like an ai_worker source tree:"
        echo "  ${AI_WORKER_SOURCE_DIR}"
        echo "Expected:"
        echo "  ffw_bringup/package.xml"
        echo "  ffw_teleop/package.xml"
        exit 1
    fi
}

ensure_container_mount() {
    local current_source
    current_source=$(docker inspect "$CONTAINER_NAME" \
        --format '{{range .Mounts}}{{if eq .Destination "/root/ros2_ws/src/ai_worker"}}{{.Source}}{{end}}{{end}}' \
        2>/dev/null || true)
    if [ -n "${current_source}" ] && [ "${current_source}" != "${AI_WORKER_SOURCE_DIR}" ]; then
        echo "Error: running container mounts a different ai_worker source tree."
        echo "  current:  ${current_source}"
        echo "  expected: ${AI_WORKER_SOURCE_DIR}"
        echo "Stop and remove it, then start again from this source tree:"
        echo "  docker stop ${CONTAINER_NAME}"
        echo "  docker rm ${CONTAINER_NAME}"
        echo "  ${SCRIPT_DIR}/container.sh start"
        exit 1
    fi
}

# Function to start the container
start_container() {
    ensure_source_tree
    ensure_container_mount

    # Set up X11 forwarding only if DISPLAY is set
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi

    echo "Starting ai_worker container..."
    # Notify if an update is available (meta package version vs GitHub latest release)
    CURRENT_VER=$(get_current_version)
    LATEST_VER=$(get_latest_version)
    if update_available "${CURRENT_VER}" "${LATEST_VER}"; then
        print_update_notice "${CURRENT_VER}" "${LATEST_VER}"
    fi

    # Copy udev rule for FTDI (U2D2)
    sudo cp "${SCRIPT_DIR}/99-u2d2.rules" /etc/udev/rules.d/99-u2d2.rules
    # Copy udev rules for AI Worker (follower/leader symlinks)
    sudo cp "${SCRIPT_DIR}/99-ai-worker.rules" /etc/udev/rules.d/99-ai-worker.rules

    # Reload udev rules
    echo "Reloading udev rules..."
    sudo udevadm control --reload-rules
    sudo udevadm trigger

    # Pull the latest images
    docker compose -f "${SCRIPT_DIR}/docker-compose.yml" pull

    # Run docker-compose
    docker compose -f "${SCRIPT_DIR}/docker-compose.yml" up -d
}

# Function to enter the container
enter_container() {
    ensure_source_tree
    ensure_container_mount

    # Set up X11 forwarding only if DISPLAY is set
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi

    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running"
        exit 1
    fi

    # Notify if an update is available (meta package version vs git tag)
    CURRENT_VER=$(get_current_version)
    GIT_VER=$(get_latest_version)
    if update_available "${CURRENT_VER}" "${GIT_VER}"; then
        print_update_notice "${CURRENT_VER}" "${GIT_VER}"
    fi

    docker exec -it "$CONTAINER_NAME" bash
}

# Function to stop the container
stop_container() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running"
        exit 1
    fi

    echo "Warning: This will stop and remove the container. All unsaved data in the container will be lost."
    read -p "Are you sure you want to continue? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker compose -f "${SCRIPT_DIR}/docker-compose.yml" down
    else
        echo "Operation cancelled."
        exit 0
    fi
}

# Get current version from meta package (ffw) package.xml
print_update_notice() {
    local current_ver="$1"
    local latest_ver="$2"
    W=52
    BAR=$(printf '%*s' $W '' | tr ' ' '=')
    LINE1="New version available: ${latest_ver} (current: ${current_ver})."
    LINE2="1. Stop Container: ./container.sh stop"
    LINE3="2. Pull Git Repo: git pull origin jazzy"
    LINE4="3. Start Container: ./container.sh start"
    echo ""
    echo "  +${BAR}+"
    printf "  |  %-$((W-2))s|\n" "$LINE1"
    printf "  |  %-$((W-2))s|\n" "$LINE2"
    printf "  |  %-$((W-2))s|\n" "$LINE3"
    printf "  |  %-$((W-2))s|\n" "$LINE4"
    echo "  +${BAR}+"
    echo ""
}

get_current_version() {
    local ver
    if [ -f "${META_PACKAGE_XML}" ]; then
        ver=$(sed -n 's/.*<version>\([^<]*\)<\/version>.*/\1/p' "${META_PACKAGE_XML}" | head -1)
    fi
    echo "${ver:-unknown}"
}

# Get latest version from GitHub releases (ROBOTIS-GIT/ai_worker)
get_latest_version() {
    local json tag
    json=$(curl -sL --connect-timeout 5 "${GITHUB_RELEASES_API}" 2>/dev/null)
    tag=$(echo "${json}" | sed -n 's/.*"tag_name":\s*"\([^"]*\)".*/\1/p' | head -1)
    # Strip optional 'v' prefix for comparison with package.xml
    if [ -n "${tag}" ]; then
        echo "${tag#v}"
    else
        echo ""
    fi
}

# Check if the latest version is newer than the current version
update_available() {
    local current="$1"
    local git_ver="$2"
    if [ -z "${git_ver}" ]; then
        return 1
    fi
    if [ "${git_ver}" = "${current}" ]; then
        return 1
    fi
    local newer
    newer=$(echo -e "${current}\n${git_ver}" | sort -V | tail -1)
    [ "${newer}" = "${git_ver}" ]
}

# Main command handling
case "$1" in
    "help")
        show_help
        ;;
    "start")
        start_container
        ;;
    "enter")
        enter_container
        ;;
    "stop")
        stop_container
        ;;
    *)
        echo "Error: Unknown command"
        show_help
        exit 1
        ;;
esac
