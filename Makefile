WORK_DIR=${PWD}
PROJECT=calibanything
DOCKER_FILE=docker/Dockerfile
DOCKER_IMAGE=bcheong/${PROJECT}:latest

DOCKER_OPTS = \
	-it \
	--rm \
	-e DISPLAY=${DISPLAY} \
	--env="NVIDIA_DRIVER_CAPABILITIES=all" \
	--env="QT_X11_NO_MITSHM=1" \
	-v /tmp:/tmp \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v /mnt/fsx:/mnt/fsx \
	-v ~/.ssh:/root/.ssh \
	-v ~/.aws:/root/.aws \
	-v ${WORK_DIR}:/share \
	--shm-size=1G \
	--ipc=host \
	--network=host \
	--pid=host \
	--privileged

docker-build:
	docker image build \
	-f $(DOCKER_FILE) \
	-t $(DOCKER_IMAGE) \
	$(DOCKER_BUILD_ARGS) .

docker-run:
	xhost +local:root;
	docker run \
	--runtime=nvidia \
	--gpus all \
	--name $(PROJECT) \
	$(DOCKER_OPTS) \
	$(DOCKER_IMAGE) bash \
	-c "cd /share; /bin/bash;"