using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using QuatLab;

namespace QuatLab.EditorTools
{
    /// <summary>
    /// 一键搭建四元数旋转演示场景：菜单 Tools/QuaternionLab/一键搭建四元数旋转演示场景
    /// 生成：轨道相机 + 方向光 + QuaternionLab（旋转立方体/轨迹/HUD 由组件运行时创建），并保存场景。
    /// </summary>
    public static class QuaternionLabSetup
    {
        [MenuItem("Tools/QuaternionLab/一键搭建四元数旋转演示场景", priority = 1)]
        public static void BuildScene()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
                return;

            EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);

            // 1) 主相机 + 轨道控制（右键旋转 / 中键平移 / 滚轮缩放 / F 聚焦）
            GameObject camGo = new GameObject("Main Camera");
            camGo.tag = "MainCamera";
            camGo.AddComponent<Camera>();
            camGo.AddComponent<AudioListener>();
            OrbitCameraController orbit = camGo.AddComponent<OrbitCameraController>();
            orbit.yaw = 35f;
            orbit.pitch = 28f;
            orbit.distance = 10f;

            // 2) 方向光
            GameObject lightGo = new GameObject("Directional Light");
            Light light = lightGo.AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 1f;
            lightGo.transform.rotation = Quaternion.Euler(50f, -30f, 0f);

            // 3) 演示对象（立方体、坐标轴、最短弧轨迹、网格、HUD 均由组件在运行时创建）
            GameObject labGo = new GameObject("QuaternionLab");
            labGo.AddComponent<QuaternionLab>();

            // 4) 保存场景
            string path = "Assets/Scenes/QuaternionLab.unity";
            EditorSceneManager.SaveScene(EditorSceneManager.GetActiveScene(), path);

            Selection.activeGameObject = labGo;
            Debug.Log("[QuaternionLab] 场景已搭建并保存：" + path +
                      "\n按 Play 进入运行模式：轴-角/四元数/欧拉三种输入，Slerp 最短弧动画与左右乘复合演示。");
        }
    }
}
