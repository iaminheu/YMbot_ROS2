/****************************************************************************
** Meta object code from reading C++ file 'rotatable_image_display.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/realsense-ros/rviz2_rotatable_image_plugin/include/rviz2_rotatable_image_plugin/rotatable_image_display.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'rotatable_image_display.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz2__displays__ImageDisplay2_t {
    QByteArrayData data[4];
    char stringdata0[77];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz2__displays__ImageDisplay2_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz2__displays__ImageDisplay2_t qt_meta_stringdata_rviz2__displays__ImageDisplay2 = {
    {
QT_MOC_LITERAL(0, 0, 30), // "rviz2::displays::ImageDisplay2"
QT_MOC_LITERAL(1, 31, 22), // "updateNormalizeOptions"
QT_MOC_LITERAL(2, 54, 0), // ""
QT_MOC_LITERAL(3, 55, 21) // "updateRotationOptions"

    },
    "rviz2::displays::ImageDisplay2\0"
    "updateNormalizeOptions\0\0updateRotationOptions"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz2__displays__ImageDisplay2[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x0a /* Public */,
       3,    0,   25,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz2::displays::ImageDisplay2::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ImageDisplay2 *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateNormalizeOptions(); break;
        case 1: _t->updateRotationOptions(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject rviz2::displays::ImageDisplay2::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz_default_plugins::displays::ImageTransportDisplay<sensor_msgs::msg::Image>::staticMetaObject>(),
    qt_meta_stringdata_rviz2__displays__ImageDisplay2.data,
    qt_meta_data_rviz2__displays__ImageDisplay2,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz2::displays::ImageDisplay2::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz2::displays::ImageDisplay2::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz2__displays__ImageDisplay2.stringdata0))
        return static_cast<void*>(this);
    return rviz_default_plugins::displays::ImageTransportDisplay<sensor_msgs::msg::Image>::qt_metacast(_clname);
}

int rviz2::displays::ImageDisplay2::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz_default_plugins::displays::ImageTransportDisplay<sensor_msgs::msg::Image>::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
