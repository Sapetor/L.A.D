from pathlib import Path
from datetime import timedelta

from .ip_config import load_network_config

BASE_DIR = Path(__file__).resolve().parent.parent

# --- Carga de red centralizada ---
NETWORK_CONFIG = load_network_config()

# Seguridad / Debug
SECRET_KEY = 'django-insecure-!px)fft@n1i7)6@2&-l@ekdxz4qz$m4u8*^f23e%-d#cm=cvqn'
DEBUG = True

ALLOWED_HOSTS = [
    'localhost',
    '127.0.0.1',
    '0.0.0.0',
    NETWORK_CONFIG.exposed_ip,
]

# Apps
INSTALLED_APPS = [
    'django.contrib.admin',
    'django.contrib.auth',
    'django.contrib.contenttypes',
    'django.contrib.sessions',
    'django.contrib.messages',
    'django.contrib.staticfiles',

    # 3rd party
    'rest_framework',
    'corsheaders',

    # Local
    "apps.learning.apps.LearningConfig",
    "workspace.apps.WorkspaceConfig",
]

REST_FRAMEWORK = {
    "DEFAULT_AUTHENTICATION_CLASSES": (
        "rest_framework_simplejwt.authentication.JWTAuthentication",
    )
}

# Middleware (orden correcto para admin)
MIDDLEWARE = [
    'django.middleware.security.SecurityMiddleware',
    'corsheaders.middleware.CorsMiddleware',                # CORS alto en la pila
    'django.contrib.sessions.middleware.SessionMiddleware', # requerido por admin
    'django.middleware.common.CommonMiddleware',
    'django.middleware.csrf.CsrfViewMiddleware',
    'django.contrib.auth.middleware.AuthenticationMiddleware',  # requerido por admin
    'django.contrib.messages.middleware.MessageMiddleware',     # requerido por admin
    'django.middleware.clickjacking.XFrameOptionsMiddleware',
]

# URLs / WSGI
ROOT_URLCONF = "core.urls"
WSGI_APPLICATION = 'core.wsgi.application'

# Templates (como tenías)
TEMPLATES = [
    {
        'BACKEND': 'django.template.backends.django.DjangoTemplates',
        'DIRS': [],  # agrega BASE_DIR / 'templates' si usas plantillas propias
        'APP_DIRS': True,
        'OPTIONS': {
            'context_processors': [
                'django.template.context_processors.request',
                'django.contrib.auth.context_processors.auth',
                'django.contrib.messages.context_processors.messages',
            ],
        },
    },
]

# DB
DATABASES = {
    'default': {
        'ENGINE': 'django.db.backends.sqlite3',
        'NAME': BASE_DIR / 'db.sqlite3',
    }
}

# Passwords
AUTH_PASSWORD_VALIDATORS = [
    {'NAME': 'django.contrib.auth.password_validation.UserAttributeSimilarityValidator'},
    {'NAME': 'django.contrib.auth.password_validation.MinimumLengthValidator'},
    {'NAME': 'django.contrib.auth.password_validation.CommonPasswordValidator'},
    {'NAME': 'django.contrib.auth.password_validation.NumericPasswordValidator'},
]

# CORS / CSRF (usando origen derivado)
CORS_ALLOW_CREDENTIALS = True
# Evitamos CORS_ALLOW_ALL_ORIGINS=True porque ya declaras listas controladas
CORS_ALLOWED_ORIGINS = [
    "http://localhost:3000",
    NETWORK_CONFIG.frontend_origin,  # p.ej. http://192.168.100.116:3000 o https si cambias scheme
]
CSRF_TRUSTED_ORIGINS = [
    "http://localhost:3000",
    NETWORK_CONFIG.frontend_origin,
]

# i18n
LANGUAGE_CODE = 'en-us'
TIME_ZONE = 'UTC'
USE_I18N = True
USE_TZ = True

# Static / Media
STATIC_URL = 'static/'
MEDIA_URL = '/images/'

STATIC_ROOT = str(BASE_DIR / 'static_collected')
MEDIA_ROOT = str(BASE_DIR / 'static' / 'images')

STATICFILES_DIRS = [
    BASE_DIR / 'static',  # asegúrate de crear esta carpeta
]

DEFAULT_AUTO_FIELD = 'django.db.models.BigAutoField'

# JWT
SIMPLE_JWT = {
    'ACCESS_TOKEN_LIFETIME': timedelta(days=5),
    'REFRESH_TOKEN_LIFETIME': timedelta(days=90),
    'ROTATE_REFRESH_TOKENS': False,
    'BLACKLIST_AFTER_ROTATION': True,
    'UPDATE_LAST_LOGIN': False,

    'ALGORITHM': 'HS256',
    'AUTH_HEADER_TYPES': ('Bearer',),
    'AUTH_HEADER_NAME': 'HTTP_AUTHORIZATION',
    'USER_ID_FIELD': 'id',
    'USER_ID_CLAIM': 'user_id',
    'AUTH_TOKEN_CLASSES': ('rest_framework_simplejwt.tokens.AccessToken',),
    'TOKEN_TYPE_CLAIM': 'token_type',

    'JTI_CLAIM': 'jti',
    'SLIDING_TOKEN_REFRESH_EXP_CLAIM': 'refresh_exp',
    'SLIDING_TOKEN_LIFETIME': timedelta(minutes=5),
    'SLIDING_TOKEN_REFRESH_LIFETIME': timedelta(days=1),
}
