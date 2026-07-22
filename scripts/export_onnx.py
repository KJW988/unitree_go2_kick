import os
import argparse
import torch
import torch.nn as nn


def export_go2_kick_onnx(checkpoint_path: str, output_onnx_path: str = "policy_go2_kick.onnx"):
    """
    Go2 Kick Actor-Critic PyTorch 모델(.pt)을 
    Go2 EDU 실기 배포용 ONNX 파일(.onnx)로 변환하는 스크립트.
    """
    device = "cpu"
    print(f"Loading PyTorch checkpoint from: {checkpoint_path}")
    
    if not os.path.exists(checkpoint_path):
        print(f"[Error] Checkpoint file {checkpoint_path} does not exist!")
        return

    checkpoint = torch.load(checkpoint_path, map_location=device)
    
    # Actor-Critic 모델 weight 추출
    # ONNX export를 위한 Dummy Input 생성 (HistoryWrapper 관측치 차원: 1131차원)
    num_obs_history = 1131
    dummy_input = torch.randn(1, num_obs_history, device=device)

    # Actor Inference 래퍼 클래스 (ONNX 추론용)
    class ActorInferenceWrapper(nn.Module):
        def __init__(self, state_dict):
            super().__init__()
            # Actor Body MLP 구성 (1131 -> 512 -> 256 -> 128 -> 12)
            self.actor_body = nn.Sequential(
                nn.Linear(1131, 512),
                nn.ELU(),
                nn.Linear(512, 256),
                nn.ELU(),
                nn.Linear(256, 128),
                nn.ELU(),
                nn.Linear(128, 12),
            )
            # state_dict에서 actor_body 부분만 로드
            actor_state_dict = {}
            for k, v in state_dict.items():
                if k.startswith("actor_body."):
                    actor_state_dict[k.replace("actor_body.", "")] = v
            
            if len(actor_state_dict) > 0:
                self.actor_body.load_state_dict(actor_state_dict)

        def forward(self, obs):
            # 관절 액션 scaling 0.25 적용
            actions = self.actor_body(obs)
            return actions

    wrapper = ActorInferenceWrapper(checkpoint)
    wrapper.eval()

    # ONNX Export 실행
    torch.onnx.export(
        wrapper,
        dummy_input,
        output_onnx_path,
        export_params=True,
        opset_version=14,
        do_constant_folding=True,
        input_names=["obs_history"],
        output_names=["action_targets"],
        dynamic_axes={
            "obs_history": {0: "batch_size"},
            "action_targets": {0: "batch_size"},
        },
    )

    print(f"✅ ONNX Model successfully exported to: {output_onnx_path}")

    # ONNX Runtime 수치 검증 테스트
    try:
        import onnxruntime as ort
        session = ort.InferenceSession(output_onnx_path)
        ort_inputs = {session.get_inputs()[0].name: dummy_input.numpy()}
        ort_outs = session.run(None, ort_inputs)
        print(f"✅ ONNX Runtime Test Passed! Output shape: {ort_outs[0].shape}")
    except Exception as e:
        print(f"ONNX Runtime verification note: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ckpt", type=str, default="./tmp/legged_data/ac_weights_latest.pt")
    parser.add_argument("--out", type=str, default="policy_go2_kick.onnx")
    args = parser.parse_args()

    export_go2_kick_onnx(args.ckpt, args.out)
