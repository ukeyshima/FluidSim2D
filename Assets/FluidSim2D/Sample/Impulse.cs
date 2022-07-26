using UnityEngine;

namespace FluidSim2D.Sample
{
    public class Impulse : MonoBehaviour
    {
        [SerializeField] private ComputeShader _computeShader;
        [SerializeField] private FluidSim2D _fluidSim2D;
        [SerializeField] private float _radius = 0.1f;
        [SerializeField] private Vector2 _position = new Vector2(0.5f, 0);

        private Vector2Int _resolution;
        private Vector2 _resolutionFloat;
        private Vector4 _texelSize;

        private int _initImpulseKernelID;
        private int _texelSizeId;
        private int _impulseID;
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
            UpdateImpulse();
        }

        private void UpdateImpulse()
        {
            _computeShader.SetVector(_texelSizeId, _texelSize);
            _computeShader.SetFloat(_radiusID, _radius);
            _computeShader.SetVector(_positionID, _position);
            _computeShader.SetTexture(_initImpulseKernelID, _impulseID, _fluidSim2D.Impulse);

            _computeShader.Dispatch(_initImpulseKernelID,
                _resolution.x / FluidSim2D.THREAD_NUM,
                _resolution.y / FluidSim2D.THREAD_NUM,
                1);
        }

        private void InitKernel()
        {
            _initImpulseKernelID = _computeShader.FindKernel("init_impulse");
        }

        private void InitShaderID()
        {
            _texelSizeId = Shader.PropertyToID("texel_size");
            _impulseID = Shader.PropertyToID("impulse");
            _radiusID = Shader.PropertyToID("radius");
            _positionID = Shader.PropertyToID("position");
        }
    }
}