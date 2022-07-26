using System;
using UnityEngine;

namespace FluidSim2D.Sample
{
    public class Renderer : MonoBehaviour
    {
        [SerializeField] private FluidSim2D _fluidSim2D;
        [SerializeField] private Mesh _quad;
        [SerializeField] private Shader _shader;
        [SerializeField] private RenderingTarget _renderingTarget;

        private Material _mat;
        private Matrix4x4 _quadMatrix = Matrix4x4.identity;
        private Vector2Int _screenResolution = new Vector2Int(0, 0);

        private void Awake()
        {
            _mat = new Material(_shader);
        }

        private void Update()
        {
            SetRenderingTarget();
            Graphics.DrawMesh(_quad, _quadMatrix, _mat, 0);

            if (_screenResolution.x == Screen.width && _screenResolution.y == Screen.height) return;
            OnChangedResolution();
        }

        private void OnChangedResolution()
        {
            _screenResolution = new Vector2Int(Screen.width, Screen.height);
            float aspectRatio = (float)_screenResolution.x / (float)_screenResolution.y;
            float height = Camera.main.orthographicSize * 2;
            float width = height * aspectRatio;
            _quadMatrix.SetTRS(Vector3.zero, Quaternion.identity, new Vector3(width, Camera.main.orthographicSize * 2, 0));
        }

        private void OnDestroy()
        {
            Destroy(_mat);
        }

        private void SetRenderingTarget()
        {
            _mat.mainTexture = _renderingTarget switch
            {
                RenderingTarget.Velocity => _fluidSim2D.Velocity,
                RenderingTarget.Density => _fluidSim2D.Density,
                RenderingTarget.Divergence => _fluidSim2D.Divergence,
                RenderingTarget.Pressure => _fluidSim2D.Pressure,
                RenderingTarget.Temperature => _fluidSim2D.Temperature,
                RenderingTarget.Obstacles => _fluidSim2D.Obstacles,
                RenderingTarget.Impulse => _fluidSim2D.Impulse,
                _ => throw new ArgumentOutOfRangeException()
            };
        }

        private enum RenderingTarget
        {
            Velocity,
            Density,
            Divergence,
            Pressure,
            Temperature,
            Obstacles,
            Impulse
        }
    }

}