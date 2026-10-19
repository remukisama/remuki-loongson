#include "lq_module_load.hpp"
#include "lq_assert.hpp"

/********************************************************************************
 * @fn      lq_module_load::lq_module_load()
 * @brief   无参默认构造函数.
 * @param   none.
 * @return  none.
 ********************************************************************************/
lq_module_load::lq_module_load()
{
}

/********************************************************************************
 * @fn      lq_module_load::lq_module_load(std::string _module)
 * @brief   有参默认构造函数.
 * @param   _module_name : 传入需要加载的模块(单个模块, 使用 string 类型).
 * @return  none.
 ********************************************************************************/
lq_module_load::lq_module_load(std::string _module)
{
    this->set_load_module(this->get_module_name(_module));
}

/********************************************************************************
 * @fn      lq_module_load::lq_module_load(std::vector<std::string> _modules)
 * @brief   有参默认构造函数.
 * @param   _module_name : 传入需要加载的模块(多个模块, 使用 vector 类型).
 * @return  none.
 ********************************************************************************/
lq_module_load::lq_module_load(std::vector<std::string> _modules)
{
    for (const auto& module : _modules) {
        this->set_load_module(module);
    }
}

/********************************************************************************
 * @fn      lq_module_load::~lq_module_load()
 * @brief   析构函数.
 * @param   none.
 * @return  none.
 ********************************************************************************/
lq_module_load::~lq_module_load()
{
    this->set_unload_all_module();
}

/********************************************************************************
 * @fn      bool lq_module_load::set_load_module(std::string _module_name)
 * @brief   加载模块.
 * @param   _module_name : 传入需要加载的模块.
 * @return  加载成功返回 true, 加载失败返回 false.
 * @note    传入的参数可以带 .ko 后缀或者带上路径.
 ********************************************************************************/
bool lq_module_load::set_load_module(const std::string _module_name)
{
    std::string cmd;
    // 查找模块文件
    cmd = "find " + this->module_path + " -name " + this->get_module_name(_module_name) + ".ko";
    cmd = this->execute_shell_commands(cmd);
    // 如果输出为空, 说明模块文件不存在
    if (cmd.empty()) {
        lq_log_warn("module %s not found", this->get_module_name(_module_name).c_str());
        return false;
    }
    // 将模块记载到加载列表中(自动避免重复加载)
    if (std::find(this->load_.begin(), this->load_.end(), _module_name) == this->load_.end())
    {
        this->load_.push_back(this->get_module_name(_module_name));
    }
    // 检查模块是否已加载
    if (this->is_module_load(_module_name))
    {
        lq_log_warn("module %s already loaded (no reload)", this->get_module_name(_module_name).c_str());
        return true;
    }
    // 加载模块
    cmd = "insmod " + cmd;
    this->execute_shell_commands(cmd);
    // 检查模块是否加载成功
    if (this->is_module_load(_module_name)) {
        return true;
    } else {
        lq_log_error("module %s load failed", this->get_module_name(_module_name).c_str());
        return false;
    }
}

/********************************************************************************
 * @fn      bool lq_module_load::set_unload_module(std::string _module_name)
 * @brief   取消加载模块.
 * @param   _module_name : 传入需要取消加载的模块.
 * @return  取消加载成功返回 true, 取消加载失败返回 false.
 * @note    传入的参数可以带 .ko 后缀或者带上路径.
 ********************************************************************************/
bool lq_module_load::set_unload_module(const std::string _module_name)
{
    std::string cmd;
    // 查找模块文件
    cmd = "find " + this->module_path + " -name " + this->get_module_name(_module_name) + ".ko";
    cmd = this->execute_shell_commands(cmd);
    // 如果输出为空, 说明模块文件不存在
    if (cmd.empty()) {
        lq_log_warn("module %s not found", this->get_module_name(_module_name).c_str());
        return false;
    }
    // 检查模块是否已加载
    if (!this->is_module_load(_module_name))
    {
        lq_log_warn("module %s not loaded", this->get_module_name(_module_name).c_str());
        return true;
    }
    // 取消加载模块
    cmd = "rmmod " + cmd;
    this->execute_shell_commands(cmd);
    // 检查模块是否取消加载成功
    if (!this->is_module_load(_module_name)) {
        // 从加载列表中移除模块
        auto vec_it = std::remove(this->load_.begin(), this->load_.end(), this->get_module_name(_module_name));
        this->load_.erase(vec_it, this->load_.end());
        return true;
    } else {
        lq_log_error("module %s unload failed", this->get_module_name(_module_name).c_str());
        return false;
    }
}

/********************************************************************************
 * @fn      void lq_module_load::set_unload_all_module()
 * @brief   卸载所有模块.
 * @param   none.
 * @return  none.
 * @note    在析构函数中会调用, 也可提前自行调用.
 ********************************************************************************/
void lq_module_load::set_unload_all_module()
{
    std::vector<std::string> modules = this->load_;
    for (auto it = modules.rbegin(); it != modules.rend(); ++it) {
        this->set_unload_module(*it);
    }
    this->load_.clear();
}

/********************************************************************************
 * @fn      bool lq_module_load::is_module_load(const std::string& _module)
 * @brief   判断模块是否已加载.
 * @param   _module : 传入需要检测的模块.
 * @return  已加载返回 true, 未加载返回 false.
 * @note    传入的参数可以带 .ko 后缀或者带上路径.
 ********************************************************************************/
bool lq_module_load::is_module_load(const std::string& _module)
{
    // -w 精确匹配模块名, 避免部分匹配
    std::string cmd = "lsmod | grep -w " + this->get_module_name(_module);
    // 执行指令
    std::string output = this->execute_shell_commands(cmd);
    // 如果输出不为空, 说明模块已加载
    return !output.empty();
}

/********************************************************************************
 * @fn      std::string lq_module_load::get_module_name(const std::string& _input)
 * @brief   提纯模块名.
 * @param   _input : 传入需要提纯的文本.
 * @return  返回提纯后的模块名.
 * @note    去除传入参数中的 .ko 后缀和前面的路径, 只保留文件名.
 ********************************************************************************/
std::string lq_module_load::get_module_name(const std::string& _input)
{
    std::string pure_name = _input;
    // 如果传入的模块名带有.ko后缀, 去除后缀
    size_t ko_pos = pure_name.find(".ko");
    if (ko_pos != std::string::npos) {
        pure_name = pure_name.substr(0, ko_pos);
    }
    // 如果传入包含路径, 去除路径, 只保留文件名
    ko_pos = pure_name.find_last_of("/");
    if (ko_pos != std::string::npos) {
        pure_name = pure_name.substr(ko_pos + 1);
    }
    return pure_name;
}

/********************************************************************************
 * @fn      std::string lq_module_load::execute_shell_commands(const std::string& _cmd)
 * @brief   执行终端 shell 指令并返回输出结果.
 * @param   _cmd : 想要执行的指令.
 * @return  执行命令后返回的数据.
 * @note    none.
 ********************************************************************************/
std::string lq_module_load::execute_shell_commands(const std::string& _cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    // 打开管道执行命令
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(_cmd.c_str(), "r"), pclose);
    if (!pipe) {
        lq_log_error("%s --> Failed to execute command", _cmd.c_str());
        return "";
    }
    // 读取命令输出
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}
