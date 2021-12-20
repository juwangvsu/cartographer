for i in {1..100}
	do
		rosparam set /savemap_node/savecloud true
		echo "rosparam set /savemap_node/savecloud true"
		sleep 0.3
	done
