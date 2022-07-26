using System;
using UnityEngine;

namespace FluidSim2D.Sample
{
    public class Obstacles : MonoBehaviour
    {
        [SerializeField] private ComputeShader _computeShader;
        [SerializeField] private FluidSim2D _fluidSim2D;
        [SerializeField] private float _radius = 0.1f;
        [SerializeField] private Vector2 _position = new Vector2(0.5f, 0.5f);

        private Vector2Int _resolution;
        private Vector2 _resolutionFloat;
        private Vector4 _texelSize;
        
        private int _initObstaclesKernelID;
        private int _texelSizeId;
        private int _obstaclesID;
        private int _radiusID;
        private int _positionID;
        
        public Vector2Int Resolution
        {
            set
            {
                _resolution = value;
                _resolutionFloat = _resolution;
                _texelSize = new Vector4(1f / _resolutionFloat.x, 1f / _resolutionFloat.y, _resolutionFloat.x, _resolutionFloat.y);
            }
        }

        private void Awake()
        {
            InitKernel();
            InitShaderID();   
        }

        private void Update()
        {
            UpdateObstacles();
        }

        private void UpdateObstacles()
        {
            _computeShader.SetVector(_texelSizeId, _texelSize);
            _computeShader.SetFloat(_radiusID, _radius);
            _computeShader.SetVector(_positionID, _position);
            _computeShader.SetTexture(_initObstaclesKernelID, _obstaclesID, _fluidSim2D.Obstacles);

            _computeShader.Dispatch(_initObstaclesKernelID,
                _resolution.x / FluidSim2D.THREAD_NUM,
                _resolution.y / FluidSim2D.THREAD_NUM,
                1);
        }

        private void InitKernel()
        {
            _initObstaclesKernelID = _computeShader.FindKernel("init_obstacles");
        }

        private void InitShaderID()
        {
            _texelSizeId = Shader.PropertyToID("texel_size");
            _obstaclesID = Shader.PropertyToID("obstacles");
            _radiusID = Shader.PropertyToID("radius");
            _positionID = Shader.PropertyToID("position");
        }
    }
}