from django.urls import path
from api.views import getPath

urlpatterns = [
    path('get-path', getPath),
]