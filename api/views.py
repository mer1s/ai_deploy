from django.http import JsonResponse
from rest_framework.decorators import api_view
from .utils import calculatePathAKI, calculatePathMICKO, calculatePathUKI, calculatePathJOCKE

@api_view(['POST'])
def getPath(request):
    agent_name = request.data.get('agent')
    prices = request.data.get('prices')

    if agent_name:
        agent_name = agent_name.lower()
        if agent_name == 'aki':
            result = calculatePathAKI(prices)
        elif agent_name == 'jocke':
            result = calculatePathJOCKE(prices)
        elif agent_name == 'micko':
            result = calculatePathMICKO(prices)
        elif agent_name == 'uki':
            result = calculatePathUKI(prices)
        else:
            result = {"error": "Unknown agent"}

        return JsonResponse(result, safe=False)
    else:
        return JsonResponse({"error": "Agent name not provided"}, status=400)

