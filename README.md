# Project Title

使用Matlab和CoppeliaSim在平滑加速路径上生成到达给定点的二自由度手臂

Using Matlab and CoppeliaSim to generate a 2-dof-arm reaching given points through a smooth-acceleration path


## 1.CoppeliaSim 建模
https://www.youtube.com/watch?v=wHrgG9ZYh24


## 2.驱动Joint
首先获取关节的句柄
```Matlab
    h = [0,0]; %get joint handle
   [r,h(1)] = sim.simxGetObjectHandle(clientID,'Joint_1',sim.simx_opmode_blocking);
   [r,h(2)] = sim.simxGetObjectHandle(clientID,'Joint_2',sim.simx_opmode_blocking);
```
然后通过牛顿插值完成轨迹规划
```Matlab
polynomial5_interpolation = Polynomial5Interpolation('Polynomial 5', yaxis_points, xaxis_points);
polynomial5_trajectory = zeros(length(t), 3); % N x 3 array: position, velocity, acceleration
for i = 1:length(t)
    polynomial5_trajectory(i,:) = polynomial5_interpolation.getPosition(t(i));
end
p_target = [0.1*t',polynomial5_trajectory(:,1)];
```
这里的 p_target 对应着每一时间单元对应的目标点；
在这里使用了多项式插值，生成加速度，速度，轨迹平滑的路径。
更多可见：
https://www.zhihu.com/tardis/zm/art/269230598?source_id=1005

## 3.求解关节角度
```Matlab
InverseKinematics(4,Rf);
Joint_1_pos = uLINK(2).q;
Joint_2_pos = uLINK(3).q;
sim.simxSetJointTargetPosition(clientID,h(1),Joint_1_pos,sim.simx_opmode_blocking);
sim.simxSetJointTargetPosition(clientID,h(2),Joint_2_pos,sim.simx_opmode_blocking);
```
并通过 sim.simxSetJointTargetPosition 赋值给对应关节
