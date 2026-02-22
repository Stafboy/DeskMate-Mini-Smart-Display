"""
Spotify Refresh Token Helper
-----------------------------
Run this script once to obtain a refresh token for the DeskMate Mini firmware.

Requirements:
    pip install requests

Usage:
    1. Fill in your CLIENT_ID and CLIENT_SECRET from the Spotify Developer Dashboard.
       https://developer.spotify.com/dashboard
    2. Make sure your app's Redirect URI is set to: http://localhost:8888/callback
    3. Run: python get_refresh_token.py
    4. A browser window will open asking you to log in and authorize the app.
    5. After approving, you'll be redirected to localhost (which will fail to load — that's fine).
    6. Copy the full URL from your browser's address bar and paste it when prompted.
    7. Your refresh token will be printed — paste it into SP_REFRESH in the firmware.
"""

import sys
import urllib.parse
import webbrowser
import requests

# -------------------------------------------------------------------
# Fill these in with your Spotify app credentials
CLIENT_ID     = "YOUR_SPOTIFY_CLIENT_ID"
CLIENT_SECRET = "YOUR_SPOTIFY_CLIENT_SECRET"
REDIRECT_URI  = "http://localhost:8888/callback"
# -------------------------------------------------------------------

SCOPE = "user-read-playback-state user-modify-playback-state"
AUTH_URL  = "https://accounts.spotify.com/authorize"
TOKEN_URL = "https://accounts.spotify.com/api/token"


def main():
    if CLIENT_ID == "YOUR_SPOTIFY_CLIENT_ID":
        print("ERROR: Please open this script and fill in CLIENT_ID and CLIENT_SECRET first.")
        sys.exit(1)

    # Step 1: Build the authorization URL and open it in the browser
    params = {
        "client_id":     CLIENT_ID,
        "response_type": "code",
        "redirect_uri":  REDIRECT_URI,
        "scope":         SCOPE,
    }
    auth_url = AUTH_URL + "?" + urllib.parse.urlencode(params)
    print("\nOpening Spotify authorization page in your browser...")
    print("If it does not open automatically, visit this URL:\n")
    print(auth_url)
    print()
    webbrowser.open(auth_url)

    # Step 2: User pastes back the redirect URL
    print("After authorizing, your browser will redirect to localhost and show an error.")
    print("That is expected. Copy the full URL from the address bar and paste it here.")
    print()
    redirected = input("Paste the full redirect URL here: ").strip()

    # Step 3: Extract the authorization code from the URL
    parsed = urllib.parse.urlparse(redirected)
    code = urllib.parse.parse_qs(parsed.query).get("code", [None])[0]
    if not code:
        print("\nERROR: Could not find an authorization code in that URL.")
        print("Make sure you copied the full URL including the '?code=...' part.")
        sys.exit(1)

    # Step 4: Exchange the code for tokens
    response = requests.post(
        TOKEN_URL,
        data={
            "grant_type":   "authorization_code",
            "code":         code,
            "redirect_uri": REDIRECT_URI,
        },
        auth=(CLIENT_ID, CLIENT_SECRET),
    )

    if response.status_code != 200:
        print(f"\nERROR: Token request failed ({response.status_code}):")
        print(response.text)
        sys.exit(1)

    data = response.json()
    refresh_token = data.get("refresh_token")

    if not refresh_token:
        print("\nERROR: No refresh token in response. Make sure your app scope is correct.")
        sys.exit(1)

    print("\n" + "=" * 60)
    print("SUCCESS! Your Spotify refresh token is:")
    print()
    print(refresh_token)
    print()
    print("Paste this into SP_REFRESH in roboeyes_v19.ino")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
