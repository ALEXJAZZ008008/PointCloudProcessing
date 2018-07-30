#ifndef PONNECTOR_H
#define PONNECTOR_H

#include <QDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QLabel>
#include <QFileDialog>
#include <QSettings>

#include <memory>

#include "src/include/PointCloudProcessingBackend.h"
#include "ui_ponnector.h"
#include "ponnector_settings.h"

using namespace std;

//! Used by Qt GUI
namespace Ui
{
class Ponnector;
}

//!
//! \class Ponnector
//! \brief The Ponnector class.
//!
class Ponnector : public QDialog
{
    Q_OBJECT

public:
    //! Constructor
    explicit Ponnector(QDialog *parent = nullptr);

    //! Destructor
    ~Ponnector();

    //! Copy and move constructos and assignment opperators,
    Ponnector(Ponnector &);
    Ponnector & operator = (Ponnector &);
    Ponnector(Ponnector &&);
    Ponnector & operator = (Ponnector &&);

    inline Ui::Ponnector * get_ui_ptr()
    {
        return m_ui_ptr;
    }

    inline int set_ui_ptr(Ui::Ponnector *ui_ptr)
    {
        m_ui_ptr = ui_ptr;

        return 1;
    }

    int ponnector_main();

    int ponnector_kill(bool);

private:

    //! Pointer to the UI namespace
    Ui::Ponnector *m_ui_ptr;

    //! Called by destructor
    //! and any other methods aiming to destruct the class
    int destructor(bool);

    int update_settings();

    //! Updates the text in the UI
    int update_output();

signals:

private slots:

    void on__psh_connec_clicked();
};

#endif // KINECTFRONTEND_H
