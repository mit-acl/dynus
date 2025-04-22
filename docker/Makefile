build: 
	docker build -t dynus:1 .

run-sim:
	xhost + && docker run  -it \
				--gpus all \
				--env="NVIDIA_DRIVER_CAPABILITIES=all" \
				--privileged \
				-e DISPLAY=${DISPLAY} \
				--network=host \
				--env="DISPLAY" \
				--env="QT_X11_NO_MITSHM=1" \
				--volume=${PWD}/gurobi.lic:/opt/gurobi/gurobi.lic:ro \
				--rm dynus:1
