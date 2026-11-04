#include "HandEyeConfig.h"

#include "PoseMath.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <cmath>

namespace {

const char* const kPoseKeys[6] = { "tx", "ty", "tz", "rx", "ry", "rz" };

bool readNumber(const QJsonObject& object, const char* key, double& value)
{
    const QJsonValue item = object.value(QLatin1String(key));
    if (!item.isDouble() || !std::isfinite(item.toDouble())) return false;
    value = item.toDouble();
    return true;
}

} // namespace

Eigen::Matrix4d HandEyeConfig::tcpToSdk() const
{
    Eigen::Matrix4d axis = Eigen::Matrix4d::Identity();
    axis.block<3, 3>(0, 0) = sensorAxis;
    return PoseMath::poseToMatrix(tcpToSensor) * axis;
}

QString HandEyeConfig::defaultPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("calibration.json"));
}

bool HandEyeConfig::load(const QString& path, HandEyeConfig& config,
    QString& message)
{
    config = HandEyeConfig();
    message.clear();

    QFile file(path);
    if (!file.exists()) {
        QString saveMessage;
        if (config.save(path, saveMessage)) {
            message = QString::fromUtf8("未找到标定文件，已按默认手眼参数生成：%1")
                .arg(QDir::toNativeSeparators(path));
        }
        else {
            message = QString::fromUtf8("未找到标定文件，使用默认手眼参数（%1）。")
                .arg(saveMessage);
        }
        return true;
    }
    if (!file.open(QIODevice::ReadOnly)) {
        message = QString::fromUtf8("无法读取标定文件：%1").arg(file.errorString());
        return false;
    }

    QJsonParseError parseError{};
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
        message = QString::fromUtf8("标定文件 JSON 格式错误：%1")
            .arg(parseError.errorString());
        return false;
    }
    const QJsonObject root = document.object();

    HandEyeConfig loaded;
    const QJsonObject handEye = root.value(QStringLiteral("handEye")).toObject();
    for (int i = 0; i < 6; ++i) {
        if (!readNumber(handEye, kPoseKeys[i], loaded.tcpToSensor[i])) {
            message = QString::fromUtf8("标定文件缺少 handEye.%1。")
                .arg(QLatin1String(kPoseKeys[i]));
            return false;
        }
    }

    if (root.contains(QStringLiteral("sensorAxis"))) {
        const QJsonArray rows = root.value(QStringLiteral("sensorAxis")).toArray();
        bool valid = rows.size() == 3;
        for (int r = 0; valid && r < 3; ++r) {
            const QJsonArray row = rows.at(r).toArray();
            valid = row.size() == 3;
            for (int c = 0; valid && c < 3; ++c) {
                valid = row.at(c).isDouble();
                loaded.sensorAxis(r, c) = row.at(c).toDouble();
            }
        }
        if (!valid) {
            message = QString::fromUtf8("sensorAxis 必须是 3×3 数组。");
            return false;
        }
        const Eigen::Matrix3d& m = loaded.sensorAxis;
        if (!(m.transpose() * m).isApprox(Eigen::Matrix3d::Identity(), 1e-6)
            || std::abs(m.determinant() - 1.0) > 1e-6) {
            message = QString::fromUtf8(
                "sensorAxis 不是右手正交阵（行列式须为 +1），请检查轴置换。");
            return false;
        }
    }

    if (root.contains(QStringLiteral("timeOffsetUs"))
        && !readNumber(root, "timeOffsetUs", loaded.timeOffsetUs)) {
        message = QString::fromUtf8("timeOffsetUs 必须是数值。");
        return false;
    }

    config = loaded;
    message = QString::fromUtf8("已加载标定文件：%1")
        .arg(QDir::toNativeSeparators(path));
    return true;
}

bool HandEyeConfig::save(const QString& path, QString& message) const
{
    QJsonObject handEye;
    for (int i = 0; i < 6; ++i) {
        handEye.insert(QLatin1String(kPoseKeys[i]), tcpToSensor[i]);
    }
    QJsonArray rows;
    for (int r = 0; r < 3; ++r) {
        rows.append(QJsonArray{ sensorAxis(r, 0), sensorAxis(r, 1), sensorAxis(r, 2) });
    }
    QJsonObject root;
    root.insert(QStringLiteral("comment"), QString::fromUtf8(
        "handEye: HiCalibrator EIH-N 结果 T_tcp1_sensor（mm / 度，固定轴 XYZ）；"
        "sensorAxis: SDK (x,0,z) 到手眼传感器系的轴置换；"
        "timeOffsetUs: 相机行时刻补偿（µs）"));
    root.insert(QStringLiteral("handEye"), handEye);
    root.insert(QStringLiteral("sensorAxis"), rows);
    root.insert(QStringLiteral("timeOffsetUs"), timeOffsetUs);

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        message = file.errorString();
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return true;
}
