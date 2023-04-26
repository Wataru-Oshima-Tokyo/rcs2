from django.shortcuts import render, HttpResponse


def index(request):
    return render(request, 'SLAMGUI/index.html')


def generic(request, template_name):
    return render(request, 'SLAMGUI/main.js', {})
