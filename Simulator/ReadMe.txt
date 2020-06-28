1. Motivation:


2. Introduction:


3. Data file architecture:
	Config // RobotSimulator的参数
	InitMap // 地图尺寸与障碍物分布
	Task // 任务点分布
	Robot_Init_Position // 机器人的初始分布
	
	Robot_Current_Position // 机器人当前位置和目标位置，这是实时维护和变化的数据

4. Code file architecture:
	底层，
		/Lib 文件夹下有需要的eigen库
		Point.h 提供点的数据结构
		BinTree.h 提供二叉树的数据结构
		Map.h 提供地图的数据结构

	二层，
		Task.h
		Robot.h
	
	三层，
		Algorithms
	
	四层
		Main function  // Statistics?and the comparison
	
5. Algorithm description




