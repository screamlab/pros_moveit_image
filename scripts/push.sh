#!/bin/bash
set -e

# Default architecture is amd64 unless overridden by --arm64
ARCH="amd64"
ECR_URL=""
IMG_NAME=""
TAG=""

usage() {
    echo "Usage: $0 -c <container_registry> -n <image_name> -t <tag> [--arm64]"
    exit 1
}

# Parse command-line parameters
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -c|--container-registry)
            ECR_URL="$2"
            shift 2
            ;;
        -n|--image-name)
            IMG_NAME="$2"
            shift 2
            ;;
        -t|--tag)
            TAG="$2"
            shift 2
            ;;
        --arm64)
            ARCH="arm64"
            shift
            ;;
        *)
            echo "Unknown parameter: $1"
            usage
            ;;
    esac
done

if [[ -z "$ECR_URL" || -z "$IMG_NAME" || -z "$TAG" ]]; then
    echo "Error: container registry, image name, and tag are required."
    usage
fi

TARGET_IMAGE="${ECR_URL}/${IMG_NAME}:${ARCH}-${TAG}"

echo "Tagging image as ${TARGET_IMAGE}..."
docker tag ${ECR_URL}/${IMG_NAME}:latest ${TARGET_IMAGE}

echo "Pushing ${TARGET_IMAGE}..."
docker push ${TARGET_IMAGE}
