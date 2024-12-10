# Importar las bibliotecas necesarias de Flask y otros módulos
from flask import Blueprint, render_template, request, flash, redirect, url_for
from .models import User  # Importar el modelo User para interactuar con la base de datos
from werkzeug.security import generate_password_hash, check_password_hash  # Para manejar el hashing de contraseñas
from . import db          # Importar la instancia de la base de datos desde el archivo __init__.py
from flask_login import login_user, login_required, logout_user, current_user  # Para manejar la autenticación de usuarios

# Crear un objeto Blueprint para el módulo de autenticación
auth = Blueprint('auth', __name__)

# Ruta para la página principal (INICIAL)
@auth.route('/main', methods=['GET'])
def main():
    # Renderiza la plantilla "main.html" y pasa el usuario actual
    return render_template("main.html", user=current_user)

# Ruta para iniciar sesión
@auth.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':  # Si el método de la solicitud es POST
        # Obtener el correo electrónico y la contraseña del formulario
        email = request.form.get('email')
        password = request.form.get('password')

        # Buscar al usuario en la base de datos por su correo electrónico
        user = User.query.filter_by(email=email).first()
        if user:  # Si el usuario existe
            # Verificar si la contraseña ingresada coincide con la almacenada
            if check_password_hash(user.password, password):
                flash('Logged in successfully!', category='success')  # Mensaje de éxito
                login_user(user, remember=True)  # Iniciar sesión para el usuario
                #return render_template("dashboard.html",user=current_user)
                return redirect(url_for('views.home'))  # Redirigir a la página DASHBOARD
            else:
                flash('Contraseña Incorrecta, por favor intenta denuevo!.', category='error')  # Mensaje de error si la contraseña es incorrecta
        else:
            flash('Este correo no esta registrado', category='error')  # Mensaje de error si el correo no existe

    # Renderizar la plantilla "login.html" en caso de GET o si hay errores
    return render_template("login.html", user=current_user)

# Ruta para cerrar sesión
@auth.route('/logout')
@login_required  # Requiere que el usuario esté autenticado
def logout():
    logout_user()  # Cerrar sesión del usuario
    return redirect(url_for('auth.login'))  # Redirigir a la página de inicio de sesión

# Ruta para registrarse (sign-up)
@auth.route('/sign-up', methods=['GET', 'POST'])
def sign_up():
    if request.method == 'POST':
        email = request.form.get('email')
        first_name = request.form.get('firstName')
        password1 = request.form.get('password1')
        password2 = request.form.get('password2')
        remember = 'remember' in request.form  # Verifica si el checkbox está marcado

        user = User.query.filter_by(email=email).first()
        if user:
            flash('Ya esta registrado este EMAIL.', category='error')
        elif len(email) < 4:
            flash('El correo debe de tener más de 3 caracteres.', category='error')
        elif len(first_name) < 2:
            flash('EL primer nombre tiene que tener más de un caracter.', category='error')
        elif password1 != password2:
            flash('Confirmación de contraseña incorrecta, no son iguales!', category='error')
        elif len(password1) < 7:
            flash('La contraseña debe de tener mínimo 7 caracteres.', category='error')
        else:
            new_user = User(email=email, first_name=first_name, password=generate_password_hash(password1, method='pbkdf2:sha256'))
            db.session.add(new_user)
            db.session.commit()
            login_user(new_user, remember=remember)  # Usa el valor del checkbox
            flash('Account created!', category='success')
            return redirect(url_for('views.home'))

    return render_template("sign_up.html", user=current_user)


@auth.route('/usuarios')
def mostrar_usuarios():
    usuarios = User.query.all()  # Obtener todos los usuarios de la base de datos
    return render_template('usuarios.html', usuarios=usuarios)  # Pasar los usuarios a la plantilla

@auth.route('/personalize', methods=['GET', 'POST'])
def personalize():
    if request.method == 'POST':
        # Obtener los datos del formulario
        first_name = request.form.get('first_name')
        residence = request.form.get('residence')
        elder_age = request.form.get('elderAge')
        elder_height = request.form.get('elderHeight')
        elder_name = request.form.get('elderName')
        password = request.form.get('password')  # Puedes agregar un campo para cambiar la contraseña si es necesario

        # Actualizar los datos del usuario actual
        if first_name:
            current_user.first_name = first_name
        if residence:
            current_user.residence = residence
        if elder_age:
            current_user.elderAge = int(elder_age) if elder_age.isdigit() else current_user.elderAge  # Asegúrate de que sea un entero
        if elder_height:
            current_user.elderHeight = elder_height
        if elder_name:
            current_user.elderName = elder_name
        if password:  # Si deseas actualizar la contraseña
            current_user.password = password=generate_password_hash(password, method='pbkdf2:sha256')

        # Guardar los cambios en la base de datos
        db.session.commit()
        flash('Your profile has been updated!', category='success')
        return redirect(url_for('auth.personalize'))

    return render_template('personalize.html', user=current_user)