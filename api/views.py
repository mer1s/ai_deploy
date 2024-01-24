from django.http import JsonResponse
from rest_framework.decorators import api_view
from .utils import calculate_path

@api_view(['POST'])
def get_path(request):
    agent_name = request.data.get('agent')
    prices = request.data.get('prices')

    if not agent_name or agent_name.lower() not in ['aki', 'jocke', 'micko', 'uki']:
        return JsonResponse({"error": "Unknown or missing agent name"}, status=400)

    agent_name = agent_name.lower()
    result = calculate_path(agent_name, prices)
    return JsonResponse(result, safe=False)
