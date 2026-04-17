 GNU nano 8.7                        linkn.py
print("\n=== LINK LIST ===")
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i) joint_name =
    info[1].decode("utf-8") link_name = info[12].decode("utf-8")
    print(f"Joint index {i}: joint='{joint_name}', child
    link='{link_name}'")
