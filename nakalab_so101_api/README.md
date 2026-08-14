# nakalab_so101_api

SO-101 を MoveIt 2 経由で操作する同期 Python API です。

Blockly などの外部実行系は `ArmControl` の次の公開 API のみを使用します。

- readiness: `wait_until_ready`, `is_move_group_ready`, `is_ik_ready`
- metadata: `get_supported_joints`, `list_group_states`, `get_group_state`
- telemetry: `get_current_joints_pose`, `get_joint_state_age`, `get_current_pose`
- motion: `move_abs_detailed`, `move_rel_detailed`, `move_to_pose_detailed`
- joints: `joint_control_detailed`, `move_groupstate_detailed`
- gripper: `gripper_control_detailed`, `open_gripper_detailed`, `close_gripper_detailed`
- cancellation: `request_cancel`, `reset_cancel_request`, `cancel_current_goal`

外部実行系では詳細 API を `wait=True` で直列に呼び出し、返された
`ArmControlResult.status` を必ず評価してください。`KeyboardInterrupt` はキャンセルを
試行した後に再送出されます。ROS context の shutdown 後は ROS entity にアクセスせず
`ROS_SHUTDOWN` として終了します。
