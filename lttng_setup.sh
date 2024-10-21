lttng create $1
lttng enable-channel --kernel kchan
lttng enable-channel --userspace uchan
lttng enable-event --kernel --channel=kchan sched_switch,sched_wakeup
lttng enable-event --userspace --channel=uchan unc_ros*
lttng add-context -u -e unc_ros_graph_generator:* -t pid -t tid
lttng list $1

