#include <torch/torch.h>
#include <torch/script.h> // TorchScript用

#include <iostream>

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_model.pt>" << std::endl;
        return 1;
    }

    std::string model_path = argv[1];
    std::cout << "Loading model from: " << model_path << std::endl;

    try {
        // モデルをロード
	torch::jit::script::Module model = torch::jit::load(model_path, torch::kCPU);  // CPU指定
        // torch::jit::script::Module model = torch::jit::load(model_path, torch::kCUDA); // CUDA指定

        // ダミー入力を作成（obs_dimに合わせる）
        int obs_dim = 45; // 自分のモデルに合わせる

        // CPU
        torch::Tensor input = torch::randn({1, obs_dim});

        // CUDA
        // model.to(torch::kCUDA);
        // torch::Tensor input = torch::randn({1, obs_dim}).to(torch::kCUDA);

        // 推論
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(input);

        torch::Tensor output = model.forward(inputs).toTensor();

        std::cout << "Action output: " << output << std::endl;
    }
    catch (const c10::Error& e) {
        std::cerr << "エラー発生: " << e.what() << std::endl;
    }

    return 0;
}
